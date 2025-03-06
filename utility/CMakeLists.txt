add_library(utility SHARED
    pid.h
    pid.cpp
    robot_message.h
    robot_message.cpp)
target_link_libraries(utility
    argos3plugin_simulator_genericrobot)