#ifndef KILOBOT_FORMATION_H
#define KILOBOT_FORMATION_H

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
#include <boost/math/distributions/beta.hpp>

// #include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>


using namespace argos;
using boost::math::beta_distribution;
using boost::math::cdf;

enum MovingStates {KILOBOT_STATE_STOP, KILOBOT_STATE_TURNING, KILOBOT_STATE_MOVING, KILOBOT_STATE_AVOIDING};

class CKilobotFormation : public CCI_Controller {

public:

    CKilobotFormation();

    virtual ~CKilobotFormation() {}

    void Init(TConfigurationNode& t_node);

    void ControlStep();

    void Reset();

    void Destroy() {}

    inline const MovingStates GetCurrentState() const {return current_state;};
    inline const bool MovingStateChanged() const {return (previous_state != current_state);};

private:
    //metodo para asignar un id numerico unico, mas conveniente que la cadena de caracteres para
    //el ancho de banda limitado a 9 bytes de los kylobots
    void static SetIdNum(CKilobotFormation* robot);
    static UInt16 id_counter;
    UInt16 id_num;
    

    CCI_DifferentialSteeringActuator* motors;
    CCI_KilobotLEDActuator* leds;
    // CCI_GroundSensor* ground_sensors;
    CCI_KilobotCommunicationSensor * com_rx;
    CCI_KilobotCommunicationActuator* com_tx;

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


    /* Generador de números aleatorios */
    CRandom::CRNG*  rng;

    //variables para metodos que declaro aqui para que no tengan que alojarse dinamicamente cuando
    //se llama a esos metodos repetidamente
    UInt32 ticks_per_second;
    UInt32 direction;

    //TODO ordernar atributos

};

#endif
