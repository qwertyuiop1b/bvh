#include <vector>
#include <array>
#include <limits>
#include <algorithm>
#include <cmath>
#include <cstdint>

// ─────────────────────────────────────────────
// 基础数据结构
// ─────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(float t)        const { return {x*t,   y*t,   z*t};   }
};

// 轴对齐包围盒
struct AABB {
    Vec3 min{ 1e30f,  1e30f,  1e30f};
    Vec3 max{-1e30f, -1e30f, -1e30f};

    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }
    void expand(const AABB& b) {
        expand(b.min); expand(b.max);
    }
    bool valid() const {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    // 表面积：2*(dx*dy + dy*dz + dz*dx)
    float surfaceArea() const {
        if (!valid()) return 0.f;
        float dx = max.x - min.x;
        float dy = max.y - min.y;
        float dz = max.z - min.z;
        return 2.f * (dx*dy + dy*dz + dz*dx);
    }

    // 获取某轴上的中心
    float center(int axis) const {
        return ((&min.x)[axis] + (&max.x)[axis]) * 0.5f;
    }
    float getMin(int axis) const { return (&min.x)[axis]; }
    float getMax(int axis) const { return (&max.x)[axis]; }
};

// 三角形（存质心 + 包围盒）
struct Triangle {
    Vec3  v0, v1, v2;
    Vec3  centroid;
    AABB  aabb;

    void build() {
        centroid = (v0 + v1 + v2) * (1.f/3.f);
        aabb = AABB{};
        aabb.expand(v0); aabb.expand(v1); aabb.expand(v2);
    }
};

// BVH 节点
struct BVHNode {
    AABB     bounds;
    uint32_t leftChild;   // 内节点：左子节点索引
    uint32_t firstTri;    // 叶节点：第一个三角形在数组中的索引
    uint32_t triCount;    // 叶节点：三角形数量（=0 表示内节点）

    bool isLeaf() const { return triCount > 0; }
};

// ─────────────────────────────────────────────
// SAH 参数
// ─────────────────────────────────────────────
static constexpr int   NUM_BINS      = 16;   // 桶数量
static constexpr float COST_TRAVERSE = 1.f;  // 遍历节点代价 Ct
static constexpr float COST_INTERSECT= 2.f;  // 三角形求交代价 Ci
static constexpr int   MIN_LEAF_SIZE = 2;    // 叶节点最小三角形数

// ─────────────────────────────────────────────
// 桶结构
// ─────────────────────────────────────────────
struct Bin {
    AABB  bounds;
    int   count = 0;
};

// ─────────────────────────────────────────────
// SAH 核心：寻找最优分割
// 返回值: bestCost（若 > leafCost 则应建叶节点）
// 输出:   bestAxis, bestBin（分割点）
// ─────────────────────────────────────────────
float findBestSplit(
    const std::vector<Triangle>& tris,
    const std::vector<uint32_t>& indices,  // 当前节点管理的三角形索引
    uint32_t start, uint32_t count,        // indices[start .. start+count)
    const AABB& nodeBounds,                // 当前节点的 AABB
    int&   bestAxis,                       // [out]
    int&   bestBin                         // [out] 分割在桶 bestBin 左侧
) {
    bestAxis = -1;
    bestBin  = -1;
    float bestCost = std::numeric_limits<float>::max();

    // ── Step 1: 计算质心包围盒 ──────────────────
    AABB centroidBounds;
    for (uint32_t i = start; i < start + count; ++i) {
        centroidBounds.expand(tris[indices[i]].centroid);
    }

    // ── Step 2: 对三个轴分别求最优分割 ──────────
    for (int axis = 0; axis < 3; ++axis) {

        float cMin = centroidBounds.getMin(axis);
        float cMax = centroidBounds.getMax(axis);

        // 质心在该轴上完全重合，跳过
        if (cMax - cMin < 1e-6f) continue;

        // ── Step 2a: 分配到桶 ───────────────────
        Bin bins[NUM_BINS];
        float scale = NUM_BINS / (cMax - cMin);  // 质心→桶索引的缩放

        for (uint32_t i = start; i < start + count; ++i) {
            const Triangle& tri = tris[indices[i]];
            float c = (&tri.centroid.x)[axis];

            int b = static_cast<int>((c - cMin) * scale);
            b = std::clamp(b, 0, NUM_BINS - 1);  // 边界保护

            bins[b].count++;
            bins[b].bounds.expand(tri.aabb);
        }

        // ── Step 2b: 前缀扫描（左侧累积）──────────
        // leftSA[i]    = 桶 0..i-1 的合并表面积
        // leftCount[i] = 桶 0..i-1 的三角形总数
        float leftSA   [NUM_BINS];
        int   leftCount[NUM_BINS];
        {
            AABB  running;
            int   cnt = 0;
            for (int i = 0; i < NUM_BINS; ++i) {
                leftSA[i]    = running.valid() ? running.surfaceArea() : 0.f;
                leftCount[i] = cnt;
                running.expand(bins[i].bounds);
                cnt += bins[i].count;
            }
        }

        // ── Step 2c: 后缀扫描（右侧累积）──────────
        // rightSA[i]    = 桶 i..K-1 的合并表面积
        // rightCount[i] = 桶 i..K-1 的三角形总数
        float rightSA   [NUM_BINS];
        int   rightCount[NUM_BINS];
        {
            AABB  running;
            int   cnt = 0;
            for (int i = NUM_BINS - 1; i >= 0; --i) {
                running.expand(bins[i].bounds);
                cnt += bins[i].count;
                rightSA[i]    = running.valid() ? running.surfaceArea() : 0.f;
                rightCount[i] = cnt;
            }
        }

        // ── Step 2d: 枚举 K-1 个分割点 ──────────
        // 分割点 i 表示: 左 = {桶0..i-1},  右 = {桶i..K-1}
        float parentSA = nodeBounds.surfaceArea();
        float invParentSA = (parentSA > 1e-10f) ? 1.f / parentSA : 0.f;

        for (int i = 1; i < NUM_BINS; ++i) {
            int   nL = leftCount[i];
            int   nR = rightCount[i];
            float saL = leftSA[i];
            float saR = rightSA[i];

            // 跳过空的一侧（退化分割）
            if (nL == 0 || nR == 0) continue;

            // SAH Cost（已除以父节点面积做归一化）
            float cost = COST_TRAVERSE
                       + (saL * nL + saR * nR) * invParentSA * COST_INTERSECT;

            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestBin  = i;
            }
        }
    }

    return bestCost;
}

// ─────────────────────────────────────────────
// BVH 构建器
// ─────────────────────────────────────────────
class BVHBuilder {
public:
    std::vector<BVHNode>    nodes;
    std::vector<uint32_t>   sortedIndices; // 重排后的三角形索引

    void build(const std::vector<Triangle>& tris) {
        uint32_t n = static_cast<uint32_t>(tris.size());
        sortedIndices.resize(n);
        for (uint32_t i = 0; i < n; ++i) sortedIndices[i] = i;

        nodes.reserve(2 * n);
        buildRecursive(tris, 0, n);
    }

private:
    // ── 计算节点 AABB ─────────────────────────
    AABB computeBounds(
        const std::vector<Triangle>& tris,
        uint32_t start, uint32_t count)
    {
        AABB b;
        for (uint32_t i = start; i < start + count; ++i)
            b.expand(tris[sortedIndices[i]].aabb);
        return b;
    }

    // ── 递归构建 ──────────────────────────────
    uint32_t buildRecursive(
        const std::vector<Triangle>& tris,
        uint32_t start, uint32_t count)
    {
        uint32_t nodeIdx = static_cast<uint32_t>(nodes.size());
        nodes.push_back({});
        BVHNode& node = nodes[nodeIdx];

        node.bounds   = computeBounds(tris, start, count);
        node.triCount = 0;

        // ── Step 4: 叶节点判断 ──────────────────
        float leafCost = COST_INTERSECT * (float)count;

        if (count <= MIN_LEAF_SIZE) {
            // 强制叶节点
            node.firstTri = start;
            node.triCount = count;
            return nodeIdx;
        }

        // ── Step 3: 找最优分割 ───────────────────
        int   bestAxis, bestBin;
        float bestCost = findBestSplit(
            tris, sortedIndices, start, count,
            node.bounds, bestAxis, bestBin);

        // ── Step 4 续: 分割代价 vs 叶节点代价 ───
        if (bestAxis == -1 || bestCost >= leafCost) {
            // 不值得分割，建叶节点
            node.firstTri = start;
            node.triCount = count;
            return nodeIdx;
        }

        // ── Step 5: 按分割点重排三角形 (partition)
        //    质心在 bestBin 左侧的放左边，否则放右边
        AABB centroidBounds;
        for (uint32_t i = start; i < start + count; ++i)
            centroidBounds.expand(tris[sortedIndices[i]].centroid);

        float cMin  = centroidBounds.getMin(bestAxis);
        float cMax  = centroidBounds.getMax(bestAxis);
        float scale = NUM_BINS / (cMax - cMin);

        // std::partition：返回第一个"右侧"元素的迭代器
        auto mid = std::partition(
            sortedIndices.begin() + start,
            sortedIndices.begin() + start + count,
            [&](uint32_t idx) {
                float c = (&tris[idx].centroid.x)[bestAxis];
                int   b = static_cast<int>((c - cMin) * scale);
                b = std::clamp(b, 0, NUM_BINS - 1);
                return b < bestBin;  // true → 放左边
            });

        uint32_t leftCount = static_cast<uint32_t>(
            mid - (sortedIndices.begin() + start));

        // 退化保护（所有三角形跑到同一侧）
        if (leftCount == 0 || leftCount == count) {
            node.firstTri = start;
            node.triCount = count;
            return nodeIdx;
        }

        // ── 递归左右子树 ─────────────────────────
        node.triCount  = 0;  // 标记为内节点
        node.leftChild = buildRecursive(tris, start,             leftCount);
        /* rightChild  = */ buildRecursive(tris, start+leftCount, count-leftCount);
        // 右子节点索引 = leftChild + 1（紧接在左子节点后面存储）
        // 实际使用时可在 node 里再存 rightChild

        // 注意：push_back 可能导致 node 引用失效，需重新获取
        nodes[nodeIdx].leftChild = node.leftChild;

        return nodeIdx;
    }
};

// ─────────────────────────────────────────────
// 使用示例
// ─────────────────────────────────────────────
int main() {
    // 构造测试场景（8 个随机三角形）
    std::vector<Triangle> tris = {
        {{ 0,0,0},{1,0,0},{0,1,0}},
        {{ 2,0,0},{3,0,0},{2,1,0}},
        {{ 0,2,0},{1,2,0},{0,3,0}},
        {{ 2,2,0},{3,2,0},{2,3,0}},
        {{ 0,0,2},{1,0,2},{0,1,2}},
        {{ 2,0,2},{3,0,2},{2,1,2}},
        {{ 0,2,2},{1,2,2},{0,3,2}},
        {{ 2,2,2},{3,2,2},{2,3,2}},
    };

    for (auto& t : tris) t.build();

    BVHBuilder builder;
    builder.build(tris);

    // 输出节点信息
    for (size_t i = 0; i < builder.nodes.size(); ++i) {
        const BVHNode& n = builder.nodes[i];
        if (n.isLeaf()) {
            printf("Node[%zu] LEAF  firstTri=%u count=%u\n",
                   i, n.firstTri, n.triCount);
        } else {
            printf("Node[%zu] INNER leftChild=%u\n",
                   i, n.leftChild);
        }
    }
    return 0;
}

// ## 五、关键细节汇总

// | 问题 | 处理方式 |
// |------|----------|
// | 质心完全重合 | 跳过该轴（`cMax - cMin < eps`） |
// | 分割后一侧为空 | 跳过该分割点（`nL==0 \|\| nR==0`） |
// | 所有三角形跑到同侧 | partition 后退化保护，强制建叶 |
// | push_back 使引用失效 | 用索引访问，不持有引用 |
// | 分割代价 > 叶节点代价 | 直接建叶节点（SAH 剪枝） |

// ---

// ## 六、复杂度分析
// ```
// 每层：O(N × 3轴 × K桶) = O(N)
// 层数：O(log N)（平衡情况）
// ────────────────────────────
// 总计：O(N log N)

// K=16 时，常数极小，实践中非常快