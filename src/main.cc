#include <SDL3/SDL.h>
#include <cstdint>
#include <sys/types.h>
#include <vector>
#include "bvh.h"
#include "camera.h"
#include "math/vector3.h"
#include "math/aabb.h"
#include "math/utils.h"
#include "ray.h"


uint32_t TRIANGLE_COUNT = 1000;
uint32_t WIDTH = 640;
uint32_t HEIGHT = 640;


int main(int argc, char* argv[]) {

    (void)argc;
    (void)argv;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("BVH", WIDTH, HEIGHT, 0);
    if (window == nullptr) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    bool is_running = true;
    SDL_Event event;

    // random triangles
    std::vector<Triangle> triangles;
    triangles.reserve(TRIANGLE_COUNT);
    for (int i = 0; i < TRIANGLE_COUNT; i++) {
        Triangle triangle;
        auto v0 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
        auto v1 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
        auto v2 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
        
        triangle.v0 = v0 * 9.f - math::Vector3f(5.0f);
        triangle.v1 = triangle.v0 + v1;
        triangle.v2 = triangle.v0 + v2;
        triangle.centroid = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f;
        triangle.aabb = math::AABB<float>(triangle.v0, triangle.v1, triangle.v2);
        triangles.push_back(triangle);
    }

    demo1::BVH bvh;
    bvh.build(triangles);

    Camera camera;
    camera.position = math::Vector3f(0, 0, -18);

    Ray ray;
    ray.origin = camera.position;

    while (is_running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                is_running = false;
            }
        }

        for (uint32_t y = 0; y < HEIGHT; y++) {
            for (uint32_t x = 0; x < WIDTH; x++) { 
                
            }
        }


        SDL_Delay(16);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}