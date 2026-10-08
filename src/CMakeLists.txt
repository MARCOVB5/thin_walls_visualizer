cmake_minimum_required(VERSION 3.14)
project(ThinWallVisualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CGAL REQUIRED)
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

add_executable(thin_wall_vis
    main.cpp
    App.cpp
    ThinWallIdentifier.cpp
)

target_compile_definitions(thin_wall_vis PRIVATE
    ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")

target_link_libraries(thin_wall_vis PRIVATE
    CGAL::CGAL
    SFML::Graphics
    SFML::Window
    SFML::System
)
