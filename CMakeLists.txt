cmake_minimum_required(VERSION 2.8.3)
project(simulation_global_planner)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            nav_msgs
            pluginlib
            roscpp
        )

remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES simulation_global_planner
    CATKIN_DEPENDS
        cmake_modules
        costmap_2d
        nav_core
        nav_msgs
        pluginlib
        roscpp
)

add_library(simulation_global_planner src/simulation_global_planner.cpp)
add_dependencies(simulation_global_planner  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_global_planner  ${catkin_LIBRARIES})

install(TARGETS simulation_global_planner 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES simulation_global_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
