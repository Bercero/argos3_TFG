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
   // light_sensor(NULL),
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
   broadcast_interval(2),
   observation_count_down(0),
   broadcast_count_down(0),
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
    motors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    leds = GetActuator<CCI_KilobotLEDActuator>("kilobot_led");
    ground_sensors = GetSensor<CCI_GroundSensor>("ground");
    com_reciber = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");
    com_transmiter = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");

    //leyendo del archivo de configuración
    TConfigurationNode experiment_conf = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "framework");
    experiment_conf = GetNode(experiment_conf,"experiment");
    GetNodeAttribute(experiment_conf, "ticks_per_second", ticks_per_second);
    // max_turning_steps = 2 * ticks_per_second; // (pi / PIN_TURN)*ticks_per_second -> media vuelta
    max_turning_steps = 5*ticks_per_second; // TODO no entiendo por que es 5


    GetNodeAttributeOrDefault(t_node, "mean_walk_duration", mean_walk_duration, mean_walk_duration);
    GetNodeAttributeOrDefault(t_node, "observation_interval", observation_interval, observation_interval);
    observation_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "broadcast_interval", broadcast_interval, broadcast_interval);
    broadcast_interval *= ticks_per_second;
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
    observation_count_down = observation_interval;
    broadcast_count_down = broadcast_interval;
    //TODO el parametro del experimento original parece ser demasiado aqui
    walking_steps = rng->Exponential(mean_walk_duration) * ticks_per_second;
    //TODO cambiar esto cuando detecten colisiones
    walking_steps = walking_steps % 3400;

    // current_state = KILOBOT_STATE_MOVING;
    current_state = KILOBOT_STATE_STOP;
    previous_state = KILOBOT_STATE_MOVING;
    motor_L = motor_R = PIN_FORWARD;
    decision = -1;
    observations_index = 0;

    messages.clear();
    if(GetId()=="k_0")
        leds->SetColor(CColor::RED);
}

/****************************************/
/****************************************/

void CKilobotBayesianDecision::ControlStep() {
    if(decision == -1 && observation_count_down-- == 0){
        Observe();
        observation_count_down = observation_interval;
    }
    //TODO iniciar el temporizador aletoriamente
    if(observations_index > 0 && broadcast_count_down-- == 0){
        if(feedback && decision != -1)
            Broadcast(decision);
        else
            Broadcast(last_observation);
        broadcast_count_down = broadcast_interval;
    }
    CheckMessages();
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
    observations_index ++;
    //TODO hacer reading un miembro del controlador para evitar la reserva dinamica de memoria, creo que si no se hace bien se puede producir memory leaking
    std::vector<Real> readings  = ground_sensors->GetReadings();
    // LOG<<"observo "<< readings[0]<<" " <<readings[1]<<std::endl;
    last_observation = readings[0];

    // LOG<<GetId()<<" observando\n";
    //calculatePosterior();
}

void CKilobotBayesianDecision::Broadcast(SInt8 obs) {
    if (GetId() != "k_0"){
        message_t* m= new message_t;
        m->data[0] = observations_index;
        // for (unsigned short int i = 1; i < 9; i++) {
        //     m->data[i]= i;
        // }
        m->data[1] = 'g';
        m->data[2] = 'u';
        m->data[3] = 'a';
        m->data[4] = 'p';
        m->data[5] = 'e';
        m->data[6] = 'r';
        m->data[7] = 'a';
        m->data[8] = 's';
        m->type = NORMAL;
        m->crc = 4;//TODO computar crc
        LOG<<GetId()<<" envia : "<<m->data[0]<<" ";
        for (UInt8 i = 1; i < 9; i++) {
            LOG<<(char)m->data[i]<<"_";
        }
        LOG<<"CRR: "<<m->crc<<"\n";
        com_transmiter->SetMessage(m);
    }

}

void CKilobotBayesianDecision::CheckMessages(){

    if(GetId() =="k_0"){

        CCI_KilobotCommunicationSensor::TPackets p = com_reciber->GetPackets();
        if(p.size() > 0){
            LOG<<GetId()<<" recibe "<<p.size()<<" mensajes"<<"\n";
            for(UInt32 i = 0; i < p.size(); i++ ){
                LOG<<"  mensaje "<< i <<": "<< p[i].Message->data[0]<<" ";
                for(int j=1; j < 9; j++){
                    LOG<<(char)p[i].Message->data[j]<<"_";
                }
                LOG<<"CRR: "<<p[i].Message->crc<<"\n";
                LOG<<"  distancia: "<<estimate_distance(& (p[i].Distance))<<"\n";
            }
        }
    }

    //
    // if(GetId()!=id){
    //     new_data = false;
    //
    //     if(messages.find(id)==messages.end())
    //     {
    //         messages[id] = index;
    //         new_data = true;
    //     }
    //     else if(messages[id] != index)
    //     {
    //         new_data = true;
    //         messages[id] = index;
    //     }

        // if(new_data)
        //     anotardato()
    // }
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
