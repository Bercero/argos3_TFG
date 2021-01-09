include_directories(${CMAKE_SOURCE_DIR}/utility)
add_library(shapebug_loop_functions MODULE
  shapebug_loop_functions.h
  shapebug_loop_functions.cpp)

target_link_libraries(shapebug_loop_functions
  argos3core_simulator
  bmp1b)
