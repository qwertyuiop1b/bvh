cmake_minimum_required(VERSION 3.16)
project(bvh)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(thirdparty/SDL)

add_executable(bvh src/main.cc)
target_include_directories(bvh PRIVATE thirdparty/SDL/include)
target_link_libraries(bvh PRIVATE SDL3::SDL3)
target_compile_definitions(bvh PRIVATE -D_SDL_MAIN_HANDLED)