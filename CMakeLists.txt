cmake_minimum_required(VERSION 3.1)

project(ASTUBOX2D VERSION 0.10.0)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${PROJECT_SOURCE_DIR}/box2d box2d)

add_library(astu_box2d 
                        src/Box2DPhysicsSystem.cpp
                        src/CBox2DBody.cpp
                        src/CBox2DColliders.cpp
            )

target_link_libraries(astu_box2d box2d)    

target_include_directories(
    astu_box2d PRIVATE "${PROJECT_SOURCE_DIR}/src"
    astu_box2d PRIVATE "${PROJECT_SOURCE_DIR}/../astu/include"
    astu_box2d PRIVATE "${PROJECT_SOURCE_DIR}/box2d/include"
    astu_box2d PUBLIC ${PROJECT_SOURCE_DIR}/include 
    )