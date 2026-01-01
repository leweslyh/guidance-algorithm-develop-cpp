# CMakeLists.txt - 项目构建配置
cmake_minimum_required(VERSION 3.15)
project(GuidanceSystem VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 寻找Eigen3库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 添加项目源文件
file(GLOB_RECURSE SOURCES
    "core/*.cpp" "core/*.hpp"
    "filter/*.cpp" "filter/*.hpp"
    "guidance/*.cpp" "guidance/*.hpp"
    "models/*.cpp" "models/*.hpp"
    "simulation/*.cpp" "simulation/*.hpp"
    "evaluation/*.cpp" "evaluation/*.hpp"
)

# 添加主程序
add_executable(guidance_simulation main.cpp ${SOURCES})

# 链接Eigen3库
target_link_libraries(guidance_simulation PRIVATE Eigen3::Eigen)

# 设置包含目录
target_include_directories(guidance_simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 添加测试目录（如果需要）
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
