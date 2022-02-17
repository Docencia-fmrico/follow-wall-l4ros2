ament_add_gtest(example_sensing_test example_sensing_test.cpp)
ament_target_dependencies(example_sensing_test ${dependencies})
target_link_libraries(example_sensing_test ${PROJECT_NAME})
