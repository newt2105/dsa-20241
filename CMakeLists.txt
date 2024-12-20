cmake_minimum_required(VERSION 3.16)

# Tên project và cấu hình chuẩn C++
project(PathFinding)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Thêm thư mục chứa file header
include_directories(${CMAKE_SOURCE_DIR}/inc)

# Thêm file nguồn
set(SOURCES
    src/main.cpp
    src/algorithms.cpp
    src/graph.cpp
)

# Tạo executable
add_executable(${PROJECT_NAME} ${SOURCES})
