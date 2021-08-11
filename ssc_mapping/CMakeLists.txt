cmake_minimum_required(VERSION 2.8.3)
project(ssc_mapping)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


catkin_package()

# cs_add_library(${PROJECT_NAME}
#         src/ssc.cpp
#         )

        
cs_add_executable(${PROJECT_NAME}_node
        src/visualization/visualization.cpp
        src/ssc.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

cs_install()
cs_export()
