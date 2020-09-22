#include "kilobot_bayesiandecision.h"

//velocidades para asignar a los motores
#define PIN_FORWARD 1.0f;
#define PIN_TURN    1.57f;
#define PIN_STOP    0.0f;

UInt16 CKilobotBayesianDecision::id_counter = 0;

CKilobotBayesianDecision::CKilobotBayesianDecision() :
   motors(NULL),
   leds(NULL),
   ground_sensors(NULL),
   com_rx(NULL),
   com_tx(NULL),
   current_state(KILOBOT_STATE_STOP),
   previous_state(KILOBOT_STATE_STOP),
   mean_walk_duration(240),
   max_turning_steps(50),
   walking_steps(1),
   turning_steps(1),
   motor_L(0.0f),
   motor_R(0.0f),
   obs_interval(300),
   obs_timer(0),
   com_interval(0.5),
   com_timer(0),
   floor_end_interval(0.5),
   floor_end_timer(0),
   credible_threshold(0.9),
   prior(1),
   feedback(true),
   decision(-1),
   obs_index(0),
   last_obs(-1),
   w_obs(prior),
   b_obs(prior)
{
   rng = CRandom::CreateRNG( "argos" );
   out_msg = new message_t;
   CKilobotBayesianDecision::SetIdNum(this);
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
    max_turning_steps = 5 * ticks_per_second;

    //TODO comprobar parametros de configuración
    // if( tal y cual) {
    //    LOGERR << "[FATAL] Invalid value for num_moving_steps (" << m_unMaxMotionSteps << "). Should be a positive integer." << std::endl;
    // }

    GetNodeAttributeOrDefault(t_node, "observation_interval", obs_interval, obs_interval);
    obs_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "broadcast_interval", com_interval, com_interval);
    com_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "avoidance_interval", floor_end_interval, floor_end_interval);
    floor_end_interval *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "mean_walk_duration", mean_walk_duration, mean_walk_duration);
    mean_walk_duration *= ticks_per_second;
    GetNodeAttributeOrDefault(t_node, "prior", prior, prior);
    GetNodeAttributeOrDefault(t_node, "feedback", feedback, feedback);
    GetNodeAttributeOrDefault(t_node, "credible_threshold", credible_threshold, credible_threshold);



    Reset();
}

/****************************************/
/****************************************/
//TODO parece que se llama dos veces cuando se pulsa reset en la interfaz grafica. Es un bug?
void CKilobotBayesianDecision::Reset() {
    obs_timer = obs_interval;
    floor_end_timer = floor_end_interval;
    com_timer = rng->Uniform(CRange<UInt32>(1, (UInt32) com_interval));
    //TODO el parametro del experimento original parece ser demasiado aqui
    walking_steps = rng->Exponential(mean_walk_duration);

    current_state = KILOBOT_STATE_MOVING;
    previous_state = KILOBOT_STATE_MOVING;
    motor_L = motor_R = PIN_FORWARD;
    decision = -1;
    obs_index = 0;
    w_obs = b_obs = prior;

    old_msgs.clear();
    leds->SetColor(CColor::PURPLE);
}

/****************************************/
/****************************************/

void CKilobotBayesianDecision::ControlStep() {

    //cada cierto tiempo se comprueba que el suelo sea gris, lo que indicaria el fin de la arena
    if(--floor_end_timer <=0){
        CheckGray();
        floor_end_timer = floor_end_interval;
    }
    //comprobar si toca hacer observación
    if(decision == -1 && --obs_timer <= 0)
    {
        CheckBlackWhite();
        obs_timer = obs_interval;
    }

    //enviar ultima observacion a los robots vecinos
    if(obs_index > 0 && --com_timer <= 0){
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
    switch(current_state) {

        case KILOBOT_STATE_TURNING:
            if( --turning_steps <= 0 ) {
                motor_L = motor_R = PIN_FORWARD;
                walking_steps = rng->Exponential(mean_walk_duration);
                previous_state = current_state;
                current_state = KILOBOT_STATE_MOVING;
            }
        break;
        case KILOBOT_STATE_AVOIDING:
            if( --walking_steps <= 0 ) {
                direction = rng->Uniform(CRange<UInt32>(0,2));
                if( direction == 0 ) {
                    motor_L = PIN_TURN;
                    motor_R = PIN_STOP;
                }
                else {
                    motor_L = PIN_STOP;
                    motor_R = PIN_TURN;
                }
                //Giro de al menos 90 grados
                turning_steps = rng->Uniform(CRange<UInt32>(max_turning_steps/2 ,max_turning_steps));
                previous_state = current_state;
                current_state = KILOBOT_STATE_TURNING;
            }
        break;
        case KILOBOT_STATE_MOVING:
            if( --walking_steps <= 0 ) {
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
                previous_state = current_state;
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
void CKilobotBayesianDecision::CheckGray() {
    std::vector<Real> readings  = ground_sensors->GetReadings();
    //detectando si ha llegado al margen de la arena
    if(current_state != KILOBOT_STATE_AVOIDING
        && previous_state != KILOBOT_STATE_AVOIDING
        && readings[1] > 0.1 && readings[1] < 0.9 ){
        previous_state = current_state;
        current_state = KILOBOT_STATE_AVOIDING;
        walking_steps = 5 * ticks_per_second;
        motor_L = motor_R = - PIN_FORWARD;//marcha atras
    }
}

void CKilobotBayesianDecision::CheckBlackWhite() {
    std::vector<Real> readings  = ground_sensors->GetReadings();
    if(readings[1] < 0.1 || readings[1] > 0.9 ){
        last_obs = readings[0];
        obs_index ++;
        AddObservation(last_obs);
    }
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

}

void CKilobotBayesianDecision::PollMessages(){

    in_msgs = com_rx->GetPackets();
    if(in_msgs.size() > 0)
    {

        for(UInt32 i = 0; i < in_msgs.size(); i++ ){
            if(decision == -1)
            {
                id_msg = (UInt16) in_msgs[i].Message->data[0];
                obs_index_msg = (UInt32) in_msgs[i].Message->data[2];
                obs_msg = in_msgs[i].Message->data[6];

                if(id_num != id_msg){
                    //comprobando si es información nueva
                    it = old_msgs.find(id_msg);
                    if( it == old_msgs.end())
                    {
                        //inserta el mensaje
                        old_msgs[id_msg] = obs_index_msg;
                        AddObservation(obs_msg);
                    }
                    else if(it->second != obs_index_msg)
                    {
                        //actualiza el mensaje guardado con el nuevo indice de observacion
                        it->second = obs_index_msg;
                        AddObservation(obs_msg);
                    }

                }
            }
            //detectando cuando si ha topado con otro kilobot
            if(current_state != KILOBOT_STATE_AVOIDING &&
                previous_state != KILOBOT_STATE_AVOIDING &&
                estimate_distance(& (in_msgs[i].Distance)) < 50)
            {
                // leds->SetColor(CColor::BLUE);
                previous_state = current_state;
                current_state = KILOBOT_STATE_AVOIDING;
                walking_steps = 5 * ticks_per_second;
                motor_L = motor_R = - PIN_FORWARD;
            }
        }
    }
}

void CKilobotBayesianDecision::AddObservation(SInt8 obs){
    w_obs += obs;
    b_obs += (1 -obs);
    p = cdf( beta_distribution<>(w_obs, b_obs), 0.5);

    if (p >= credible_threshold){
        decision = 0;
        LOG<<GetId()<<"decide NEGRO ";
        LOG<<" white:"<<w_obs<<" black:"<<b_obs;
        LOG<<" p = "<< p<<std::endl;
        leds->SetColor(CColor::RED);
    }
    else if ((1 -p) >= credible_threshold){
        decision = 1;
        LOG<<GetId()<<"decide BLANCO ";
        LOG<<" white: "<<w_obs<<" black: "<<b_obs;
        LOG<<" p = "<< p<<std::endl;
        leds->SetColor(CColor::GREEN);
    }

}
void CKilobotBayesianDecision::SetIdNum(CKilobotBayesianDecision* robot){
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
