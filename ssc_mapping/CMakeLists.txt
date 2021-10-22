cmake_minimum_required(VERSION 2.8.3)
project(ssc_mapping)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


catkin_package()

cs_add_library(${PROJECT_NAME}
        src/visualization/visualization.cpp
        src/core/ssc_map.cpp
        src/ros/ssc_server.cpp
        src/fusion/occupancy_fusion.cpp
        src/fusion/log_odds_fusion.cpp
        src/fusion/counting_fusion.cpp
        )

        
cs_add_executable(${PROJECT_NAME}_node
        src/ssc_server_node.cpp
)

cs_add_executable(ssc_map_eval_node
        src/eval/ssc_map_eval_node.cpp
)

cs_add_executable(ssc_map_eval_test_node
        src/eval/ssc_map_eval_test_node.cpp
)

cs_add_executable(fill_ground_truth_map_node
        src/eval/fill_ground_truth_map_node.cpp
)


target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries(ssc_map_eval_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries(ssc_map_eval_test_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries(fill_ground_truth_map_node ${PROJECT_NAME} ${catkin_LIBRARIES} )

cs_install()
cs_export()
