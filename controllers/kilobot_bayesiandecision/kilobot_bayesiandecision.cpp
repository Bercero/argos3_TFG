#include "kilobot_bayesiandecision.h"

/****************************************/
/****************************************/

//velocidades para asignar a los motores
#define PIN_FORWARD 1.0f;
#define PIN_TURN    1.57f;
#define PIN_STOP    0.0f;

UInt16 CKilobotBayesianDecision::id_counter = 0;

CKilobotBayesianDecision::CKilobotBayesianDecision() :
   motors(NULL),
   leds(NULL),
   // light_sensor(NULL),
   current_state(KILOBOT_STATE_STOP),
   previous_state(KILOBOT_STATE_STOP),
   max_turning_steps(50),
   turning_steps(1),
   decision(-1),
   motor_L(0.0f),
   motor_R(0.0f),
   obs_index(0),
   walking_steps(0),
   mean_walk_duration(240),
   obs_interval(300),
   com_interval(0.5),
   obs_timer(0),
   com_timer(0),
   prior(25),
   feedback(true),
   last_obs(-1)
{
   rng = CRandom::CreateRNG( "argos" );
   out_msg = new message_t;
   CKilobotBayesianDecision::setIdNum(this);
}

/****************************************/
/* Se leen lor parametros de confinguración*/
/****************************************/

void CKilobotBayesianDecision::Init(TConfigurationNode& t_node) {
    motors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    leds = GetActuator<CCI_KilobotLEDActuator>("kilobot_led");
    ground_sensors = GetSensor<CCI_GroundSensor>("ground");
    com_rx = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");
    com_tx = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");

    //leyendo del archivo de configuración
    TConfigurationNode experiment_conf = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "framework");
    experiment_conf = GetNode(experiment_conf,"experiment");
    GetNodeAttribute(experiment_conf, "ticks_per_second", ticks_per_second);
    // max_turning_steps = 2 * ticks_per_second; // (pi / PIN_TURN)*ticks_per_second -> media vuelta
    max_turning_steps = 5*ticks_per_second; // TODO no entiendo por que es 5


    GetNodeAttributeOrDefault(t_node, "mean_walk_duration", mean_walk_duration, mean_walk_duration);
    GetNodeAttributeOrDefault(t_node, "observation_interval", obs_interval, obs_interval);
    obs_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "broadcast_interval", com_interval, com_interval);
    com_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "prior", prior, prior);
    GetNodeAttributeOrDefault(t_node, "feedback", feedback, feedback);

    //TODO comprobar parametros de configuración
    // if( tal y cual) {
    //    LOGERR << "[FATAL] Invalid value for num_moving_steps (" << m_unMaxMotionSteps << "). Should be a positive integer." << std::endl;
    // }

    Reset();
}

/****************************************/
/****************************************/
//TODO parece que se llama dos veces cuando se pulsa reset en la interfaz grafica. Es un bug?
void CKilobotBayesianDecision::Reset() {
    obs_timer = obs_interval;
    com_timer = com_interval;
    //TODO el parametro del experimento original parece ser demasiado aqui
    walking_steps = rng->Exponential(mean_walk_duration) * ticks_per_second;
    //TODO cambiar esto cuando detecten colisiones
    walking_steps = walking_steps % 3400;

    // current_state = KILOBOT_STATE_MOVING;
    current_state = KILOBOT_STATE_STOP;
    previous_state = KILOBOT_STATE_MOVING;
    motor_L = motor_R = PIN_FORWARD;
    decision = -1;
    obs_index = 0;

    old_msgs.clear();
    if(GetId()=="k_0")
        leds->SetColor(CColor::RED);
}

/****************************************/
/****************************************/

void CKilobotBayesianDecision::ControlStep() {
    if(decision == -1 && obs_timer-- <= 0){
        Observe();
        obs_timer = obs_interval;
    }
    //TODO iniciar el temporizador aletoriamente
    if(obs_index > 0 && com_timer-- <= 0){
        if(feedback && decision != -1)
            Broadcast(decision);
        else
            Broadcast(last_obs);
        com_timer = com_interval;
    }
    PollMessages();
    // paseo aleatorio
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
    obs_index ++;
    std::vector<Real> readings  = ground_sensors->GetReadings();
    LOG<<GetId()<<" observa "<< readings[0]<<" " <<readings[1]<<std::endl;
    last_obs = readings[0];

    //calculatePosterior();
}

void CKilobotBayesianDecision::Broadcast(SInt8 obs) {
    //contruccion del mensaje:

    //id_num en los dos primeros bytes del mensajes
    byte_ptr = (UInt8 *)&id_num;
    out_msg->data[0] = byte_ptr[0];
    out_msg->data[1] = byte_ptr[1];

    //despues va el número de observaciones
    byte_ptr = (UInt8 *)&obs_index;
    out_msg->data[2] = byte_ptr[0];
    out_msg->data[3] = byte_ptr[1];
    out_msg->data[4] = byte_ptr[2];
    out_msg->data[5] = byte_ptr[3];

    //por ultimo el dato
    out_msg->data[6] = obs;

    out_msg->type = NORMAL;
    out_msg->crc = 4;//TODO computar crc, no es necesario de momento, pero puede que cambiar al actualizarse el plugin de los kilobots

    com_tx->SetMessage(out_msg);
    LOG<<id_num<<" envia "<< obs_index << " "<< obs<<std::endl;

}

void CKilobotBayesianDecision::PollMessages(){

    if(GetId() =="k_0"){
        in_msgs = com_rx->GetPackets();
        if(in_msgs.size() > 0){
            LOG<<id_num<<" recibe "<<in_msgs.size()<<" mensajes"<<std::endl;
            for(UInt32 i = 0; i < in_msgs.size(); i++ ){
                id_msg = (UInt16 *) in_msgs[i].Message->data;
                obs_index_msg = (UInt32 *) (in_msgs[i].Message->data + 2);
                obs_msg = in_msgs[i].Message->data[6];
                LOG<<"___: "<< *id_msg<<" "<<*obs_index_msg<< " "<< obs_msg<<std::endl;
            }
            // estimate_distance(& (in_msgs[i].Distance));
        }


        // if(id_num != id_msg){
        //     new_data = false;
        //     it = old_msgs.find(id_msg)
        //     if( it == old_msgs.end())
        //     {
        //         old_msgs[id_msg] = ;
        //         new_data = true;
        //     }
        //     else if(old_msgs[id] != index)
        //     {
        //         new_data = true;
        //         old_msgs[id] = index;
        //     }
        //
        //     if(new_data)
        //         anotardato()
        // }
    }
}
void CKilobotBayesianDecision::setIdNum(CKilobotBayesianDecision* robot){
    robot->id_num = id_counter ++;
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
