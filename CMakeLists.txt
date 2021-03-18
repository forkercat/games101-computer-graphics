cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(Assignment5
    assignment5/main.cpp
    assignment5/Object.hpp
    assignment5/Vector.hpp
    assignment5/Sphere.hpp
    assignment5/global.hpp
    assignment5/Triangle.hpp
    assignment5/Scene.cpp
    assignment5/Scene.hpp
    assignment5/Light.hpp
    assignment5/Renderer.cpp)
target_compile_options(Assignment5 PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
target_compile_features(Assignment5 PUBLIC cxx_std_17)
target_link_libraries(Assignment5 PUBLIC -fsanitize=undefined)
