cmake_minimum_required(VERSION 3.6) # CMake install : https://cmake.org/download/

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

set(CMAKE_CXX_STANDARD 17)

# # 调试信息
# add_compile_definitions(BUILD_TYPE)


# 添加第三方库 libssh2
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libssh2 libssh2)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libssh2/include)

# 添加项目
add_subdirectory(./core)
# add_subdirectory(./example)
