cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(mpc_dummy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  OptimisationResult.msg
  OptimisationParameters.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_talker src/talker.cpp)
target_link_libraries(${PROJECT_NAME}_talker /home/chung/catkin_ws/src/mpc_dummy/extern_lib/librosenbrock.a)
target_link_libraries(${PROJECT_NAME}_talker m)
target_link_libraries(${PROJECT_NAME}_talker dl)
target_link_libraries(${PROJECT_NAME}_talker pthread)
target_link_libraries(${PROJECT_NAME}_talker ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_talker mpc_dummy_generate_messages_cpp)
