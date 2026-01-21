cmake_minimum_required(VERSION 3.5)
project(coffee_detect)

# 默认编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 和 ament_cmake_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # ← 必须添加这一行！

# 安装 Python 模块（保持包结构）
ament_python_install_package(${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# 安装 config 文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# 安装 sam 目录
# install(DIRECTORY sam/
#  DESTINATION share/${PROJECT_NAME}/sam)

# 安装可执行脚本（关键！）
install(PROGRAMS
  coffee_detect/coffee.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出
ament_package()