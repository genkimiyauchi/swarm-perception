link_directories(${CMAKE_BINARY_DIR}/controllers/robot)
set(target_tracking_loop_functions_SOURCES target_tracking_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(target_tracking_loop_functions_SOURCES
    ${target_tracking_loop_functions_SOURCES}
    target_tracking_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(target_tracking_loop_functions MODULE ${target_tracking_loop_functions_SOURCES})
target_link_libraries(target_tracking_loop_functions
  robot
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(target_tracking_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
