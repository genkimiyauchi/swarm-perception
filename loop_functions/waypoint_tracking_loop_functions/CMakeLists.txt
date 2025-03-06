link_directories(${CMAKE_BINARY_DIR}/controllers/epuck_waypoint_tracking)
set(waypoint_tracking_loop_functions_SOURCES waypoint_tracking_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(waypoint_tracking_loop_functions_SOURCES
    ${waypoint_tracking_loop_functions_SOURCES}
    # waypoint_tracking_qt_user_functions.cpp)
  )
endif(ARGOS_QTOPENGL_FOUND)

add_library(waypoint_tracking_loop_functions MODULE ${waypoint_tracking_loop_functions_SOURCES})
target_link_libraries(waypoint_tracking_loop_functions
  epuck_waypoint_tracking
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(waypoint_tracking_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
