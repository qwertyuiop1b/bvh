#include <SDL3/SDL.h>
#include <chrono>
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

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (renderer == nullptr) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
    if (texture == nullptr) {
        SDL_Log("SDL_CreateTexture failed: %s", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    std::vector<uint32_t> pixels(WIDTH * HEIGHT, 0);
    const SDL_PixelFormatDetails* format = SDL_GetPixelFormatDetails(SDL_PIXELFORMAT_RGBA8888);

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
        
        triangle.v0 = v0 * 10.f - math::Vector3f(5.0f);
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
    math::Vector3f p0(-1, 1, -15), p1(1 , 1, -15), p2(-1, -1, -15);
    ray.origin = camera.position;

    auto start = std::chrono::high_resolution_clock::now();

    for (uint32_t y = 0; y < HEIGHT; y++) {
        for (uint32_t x = 0; x < WIDTH; x++) { 
            math::Vector3f pixelPos = p0 + (p1 - p0) * (x / float(WIDTH)) + (p2 - p0) * (y / float(HEIGHT));
            ray.direction = math::normalize(pixelPos - camera.position);
            ray.t = 1e30f;

            for (int i = 0; i < triangles.size(); i++) {
                bvh.intersectTriangle(ray, triangles[i]);
            }

            uint8_t c = 0;
            if (ray.t < 1e30f)  {
                c = 255;
            }
            pixels[y * WIDTH + x] = SDL_MapRGBA(format, nullptr, c, c, c, 255);

        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
    SDL_Log("Rasterization time: %.2f ms", duration_ms);

    demo2::BVH bvh2;
    bvh2.createTriangles(TRIANGLE_COUNT);
    bvh2.build();
    
    auto start2 = std::chrono::high_resolution_clock::now();
    for (uint32_t y = 0; y < HEIGHT; y++) {
        for (uint32_t x = 0; x < WIDTH; x++) { 
        math::Vector3f pixelPos = p0 + (p1 - p0) * (x / float(WIDTH)) + (p2 - p0) * (y / float(HEIGHT));
        ray.direction = math::normalize(pixelPos - camera.position);
        ray.t = 1e30f;

        bvh2.intersectBVH(ray, 0);

        uint8_t c = 0;
        if (ray.t < 1e30f)  {
            c = 255;
        }
        pixels[y * WIDTH + x] = SDL_MapRGBA(format, nullptr, c, c, c, 255);

        }
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration_ms2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end2 - start2).count();
    SDL_Log("Rasterization with BVH time: %.2f ms", duration_ms2);


    while (is_running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                is_running = false;
            }
        }

        SDL_UpdateTexture(texture, nullptr, pixels.data(), WIDTH * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(16);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}