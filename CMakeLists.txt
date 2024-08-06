cmake_minimum_required(VERSION 3.10)

project(NBodySimulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Add source files
add_executable(NBodySimulation main.cpp)

# Find and link GLFW
find_package(GLFW3 REQUIRED)
target_link_libraries(NBodySimulation PRIVATE glfw)

# Include directories for ImGui and GLAD
target_include_directories(NBodySimulation PRIVATE imgui glad/include)

# Add ImGui source files
file(GLOB IMGUI_SOURCES
        "imgui/*.cpp"
        "imgui/imgui_impl_glfw.cpp"
        "imgui/imgui_impl_opengl3.cpp"
)

# Add GLAD source files
file(GLOB GLAD_SOURCES
        "glad/src/glad.c"
)

# Add ImGui and GLAD source files to the executable
target_sources(NBodySimulation PRIVATE ${IMGUI_SOURCES} ${GLAD_SOURCES})

# Find and link OpenGL
find_package(OpenGL REQUIRED)
target_link_libraries(NBodySimulation PRIVATE OpenGL::GL)