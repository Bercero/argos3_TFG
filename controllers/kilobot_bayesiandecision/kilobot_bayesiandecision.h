#ifndef KILOBOT_BAYESIANDECISION_H
#define KILOBOT_BAYESIANDECISION_H

#include <map>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_led_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
#include <argos3/core/utility/logging/argos_log.h>
// #include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>


using namespace argos;

enum MovingStates {KILOBOT_STATE_STOP, KILOBOT_STATE_TURNING, KILOBOT_STATE_MOVING};

class CKilobotBayesianDecision : public CCI_Controller {

public:

   CKilobotBayesianDecision();

   virtual ~CKilobotBayesianDecision() {}

   void Init(TConfigurationNode& t_node);

   void ControlStep();

   void Reset();

   void Destroy() {}

   void Observe();

   void Broadcast(SInt8 obs);

   void CheckMessages();

   inline const SInt8 GetDecision() const {return decision;}
   inline const MovingStates GetCurrentState() const {return current_state;};
   inline const bool MovingStateChanged() const {return (previous_state != current_state);};

private:

    CCI_DifferentialSteeringActuator* motors;
    CCI_KilobotLEDActuator* leds;
    // CCI_KilobotLightSensor* light_sensor;
    CCI_GroundSensor* ground_sensors;
    CCI_KilobotCommunicationSensor * com_reciber;
    CCI_KilobotCommunicationActuator* com_transmiter;

    /* estados para controlar el paseo aleatorio*/
    MovingStates current_state;
    MovingStates previous_state; //TODO no se usa este valor

    /* Paramentros y contadores del paseo aleatorio */
    Real mean_walk_duration;
    UInt32 max_turning_steps;

    UInt32 walking_steps;
    UInt32 turning_steps;


    /* velocidades de los motores */
    Real   motor_L;
    Real   motor_R;

    UInt32 broadcast_interval;
    UInt32 broadcast_count_down;

    //parametros relacionados con las observaciones y el modelo estadistico de decision
    UInt32 observation_interval;
    UInt32 observation_count_down;
    UInt32 observations_index;
    SInt8 last_observation;
    SInt8 decision;
    Real prior;
    bool feedback;

    //diccionario para saber cual es la ultima observacion recibida de cada robot
    std::map<std::string, UInt32> messages;

    /* Generador de números aleatorios */
    CRandom::CRNG*  rng;

    //variables para metodos que declaro aqui para que no tengan que alojarse dinamicamente cuando
    //se llama a esos metodos
    UInt32 ticks_per_second;
    UInt32 direction;
    SInt16 reading;
    bool new_data;

};

#endif
