cmake_minimum_required(VERSION 3.5)

project(locobot_bringup)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
)
