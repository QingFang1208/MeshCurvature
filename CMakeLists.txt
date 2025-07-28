cmake_minimum_required(VERSION 3.5)

project(MeshCurvature)

#设置不生成MinSizeRel;RelWithDebInfo
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

############################################################
# Create a library
############################################################

#Generate the static library from the library sources
add_library(ACAM_mesh STATIC 
    src/IOManger.cpp
	src/PolyMesh.cpp
	src/PolyMesh_Base.cpp
	src/PolyMeshIterators.cpp
)

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #设置库文件的输出目录

target_include_directories(ACAM_mesh
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)

############################################################
# Project Homework
############################################################

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW main.cpp)
target_include_directories(${PROJECT_NAME}_HW
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW
    PRIVATE 
        ACAM_mesh
)