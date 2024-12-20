cmake_minimum_required(VERSION 3.16)

# Tên project và cấu hình chuẩn C++
project(PathFinding)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Thêm thư mục chứa file header
include_directories(${CMAKE_SOURCE_DIR}/inc)

file(GLOB SOURCES "src/*.cpp")

# Tạo executable
add_executable(${PROJECT_NAME} ${SOURCES})
