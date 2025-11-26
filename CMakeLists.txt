# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.26)


# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()


# 设置项目名称
project ("DMSDK")


# 设置版本号
set(MAJOR_VAR 1)
set(MINOR_VAR 2)
set(PATCH_VAR 0)
set(REVISION_VAR 0)
# 设置优化等级
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")


# 定义自定义宏
add_compile_definitions(MAJOR_VERSION=${MAJOR_VAR})
add_compile_definitions(MINOR_VERSION=${MINOR_VAR})
add_compile_definitions(PATCH_VERSION=${PATCH_VAR})
add_compile_definitions(REVISION_VERSION=${REVISION_VAR})
add_compile_definitions(LW_INTERFACE_EXPORT)
add_compile_definitions(NOMINMAX)
add_compile_definitions(LW_ZHONGRILONG_PROJ)


# 包含子项目。
include_directories("DM_SDK_C")
add_subdirectory ("DM_SDK_C")


# 将源代码添加到此项目的可执行文件。
add_executable (DMSDK "main.cpp")


# 链接库
target_link_libraries(DMSDK dm_c_sdk)
