cmake_minimum_required(VERSION 3.10)
project(BoidsSimulation)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})

add_executable(boids boids.cpp)
target_link_libraries(boids ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
