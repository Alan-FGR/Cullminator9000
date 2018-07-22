cmake_minimum_required(VERSION 3.9)
project(culler)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(Bigged)

link_libraries(
        bigged
        )

include_directories(culler
        PUBLIC
        Bigged
        Bigged/bigg/include
        Bigged/entt/src/entt
        )

add_executable(culler
        main.cpp
        )