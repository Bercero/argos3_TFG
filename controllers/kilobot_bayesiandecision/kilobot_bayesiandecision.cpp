#include "kilobot_bayesiandecision.h"

/****************************************/
/****************************************/

//velocidades para asignar a los motores
#define PIN_FORWARD 1.0f;
#define PIN_TURN    1.57f;
#define PIN_STOP    0.0f;


CKilobotBayesianDecision::CKilobotBayesianDecision() :
   motors(NULL),
   leds(NULL),
   light_sensor(NULL),
   current_state(KILOBOT_STATE_STOP),
   previous_state(KILOBOT_STATE_STOP),
   max_turning_steps(50),
   turning_steps(1),
   decision(-1),
   motor_L(0.0f),
   motor_R(0.0f),
   observations_index(0),
   walking_steps(0),
   mean_walk_duration(240),
   observation_interval(300),
   observation_count_down(300),
   prior(25),
   feedback(true),
   last_observation(-1)
{
   rng = CRandom::CreateRNG( "argos" );
}

/****************************************/
/* Se leen lor parametros de confinguración*/
/****************************************/

void CKilobotBayesianDecision::Init(TConfigurationNode& t_node) {

    light_sensor = GetSensor<CCI_KilobotLightSensor>("kilobot_light");
    motors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    leds = GetActuator<CCI_KilobotLEDActuator>("kilobot_led");

    //leyendo del archivo de configuración
    TConfigurationNode experiment_conf = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "framework");
    experiment_conf = GetNode(experiment_conf,"experiment");
    GetNodeAttribute(experiment_conf, "ticks_per_second", ticks_per_second);
    // max_turning_steps = 2 * ticks_per_second; // (pi / PIN_TURN)*ticks_per_second -> media vuelta
    max_turning_steps = 5*ticks_per_second; // TODO no entiendo por que es 5

    GetNodeAttributeOrDefault(t_node, "mean_walk_duration", mean_walk_duration, mean_walk_duration);
    GetNodeAttributeOrDefault(t_node, "observation_interval", observation_interval, observation_interval);
    observation_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "prior", prior, prior);
    GetNodeAttributeOrDefault(t_node, "feedback", feedback, feedback);

    //TODO comprobar parametros de configuración
    // if( m_unMaxMotionSteps == 0 ) {
    //    LOGERR << "[FATAL] Invalid value for num_moving_steps (" << m_unMaxMotionSteps << "). Should be a positive integer." << std::endl;
    // }

    Reset();
}

/****************************************/
/****************************************/
//TODO parece que se llama dos veces cuando se pulsa reset en la interfaz grafica. Es un bug?
void CKilobotBayesianDecision::Reset() {
    observation_count_down = observation_interval;
    //TODO el parametro del experimento original parece ser demasiado aqui
    walking_steps = rng->Exponential(mean_walk_duration) * ticks_per_second;
    //TODO cambiar esto cuando detecten colisiones
    walking_steps = walking_steps % 3400;

    current_state = KILOBOT_STATE_MOVING;
    previous_state = KILOBOT_STATE_MOVING;
    motor_L = motor_R = PIN_FORWARD;
    decision = -1;
    observations_index = 0;

    messages.clear();
    leds->SetColor(CColor::RED);
}

/****************************************/
/****************************************/

void CKilobotBayesianDecision::ControlStep() {
    if(decision == -1 && --observation_count_down == 0){
        Observe();
        observation_count_down = observation_interval;
    }
    if(observations_index > 0){
        if(feedback && decision != -1)
            Broadcast(decision);
        else
            Broadcast(last_observation);
    }

   // time, and rotate cw/ccw for a random amount of time
   // max rotation: 180 degrees as determined by max_turning_steps
   previous_state = current_state;
   switch(current_state) {
   case KILOBOT_STATE_TURNING:
      if( --turning_steps == 0 ) {
         motor_L = motor_R = PIN_FORWARD;
         walking_steps = rng->Exponential(mean_walk_duration) * ticks_per_second;
         current_state = KILOBOT_STATE_MOVING;
      }
      break;

   case KILOBOT_STATE_MOVING:
      if( --walking_steps == 0 ) {
         direction = rng->Uniform(CRange<UInt32>(0,2));
         if( direction == 0 ) {
            motor_L = PIN_TURN;
            motor_R = PIN_STOP;
         }
         else {
            motor_L = PIN_STOP;
            motor_R = PIN_TURN;
         }
         turning_steps = rng->Uniform(CRange<UInt32>(0,max_turning_steps));
         current_state = KILOBOT_STATE_TURNING;
      }
      break;

   case KILOBOT_STATE_STOP:
   default:
      motor_L = motor_R = PIN_STOP;
      break;
   };

   motors->SetLinearVelocity(motor_L, motor_R);
}

/****************************************/
/****************************************/
void CKilobotBayesianDecision::Observe() {
    observations_index ++;
    // reading = light_sensor->GetReading();
    // LOG<<reading<<std::endl;
    LOG<<"Castrochachos al poder"<<std::endl;
    // last_observation = ;

    // LOG<<GetId()<<" observando\n";
    //calculatePosterior();
}

void CKilobotBayesianDecision::Broadcast(SInt8 message) {
    // LOG<<GetId()<<" transmite "<< observations_index << ":" << message <<"\n";
}

void CKilobotBayesianDecision::Recibe(std::string id,UInt32 index, SInt8 obs){
    if(GetId()!=id){
        new_data = false;

        if(messages.find(id)==messages.end())
        {
            messages[id] = index;
            new_data = true;
        }
        else if(messages[id] != index)
        {
            new_data = true;
            messages[id] = index;
        }

        // if(new_data)
        //     anotardato()
    }
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.  The string is then usable in the configuration
 * file to refer to this controller.  When ARGoS reads that string in
 * the configuration file, it knows which controller class to
 * instantiate.  See also the configuration files for an example of
 * how this is used.
 */
REGISTER_CONTROLLER(CKilobotBayesianDecision, "kilobot_bayesiandecision_controller")
