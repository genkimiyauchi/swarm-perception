add_library(robot SHARED robot.h robot.cpp)
target_link_libraries(robot
  utility
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
