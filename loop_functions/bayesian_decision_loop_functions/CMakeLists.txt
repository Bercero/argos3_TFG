add_library(bayesian_decision_loop_functions MODULE
  bayesian_decision_loop_functions.h
  bayesian_decision_loop_functions.cpp)

target_link_libraries(bayesian_decision_loop_functions
  argos3core_simulator)

#TODO esta linea incorpora es para que las loop_functions encuentren al controller, pero parece
#que no deberia haber tanto acoplamiento entre controllers y loop_functions
include_directories(${CMAKE_SOURCE_DIR}/controllers/kilobot_bayesiandecision)
