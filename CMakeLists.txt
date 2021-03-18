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



add_executable(Assignment6
    assignment6/main.cpp
    assignment6/Object.hpp
    assignment6/Vector.cpp assignment6/Vector.hpp
    assignment6/Sphere.hpp assignment6/global.hpp
    assignment6/Triangle.hpp
    assignment6/Scene.cpp assignment6/Scene.hpp
    assignment6/Light.hpp assignment6/AreaLight.hpp
    assignment6/BVH.cpp assignment6/BVH.hpp assignment6/Bounds3.hpp
    assignment6/Ray.hpp assignment6/Material.hpp assignment6/Intersection.hpp
    assignment6/Renderer.cpp assignment6/Renderer.hpp)
