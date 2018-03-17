cmake_minimum_required(VERSION 2.8.3)
project(piko_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES
                  Dock.msg
                 )
add_service_files(FILES
                  RequestDock.srv
                 )

generate_messages(   
	DEPENDENCIES  
	std_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
