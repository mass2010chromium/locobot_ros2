cmake_minimum_required(VERSION 3.5)

project(locobot_description)

find_package(ament_cmake REQUIRED)

ament_package()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}
		DESTINATION share/${PROJECT_NAME}/)
endforeach(dir)
