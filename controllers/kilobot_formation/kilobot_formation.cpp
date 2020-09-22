#include "kilobot_formation.h"

//velocidades para asignar a los motores
#define PIN_FORWARD 1.0f;
#define PIN_TURN    1.57f;
#define PIN_STOP    0.0f;

UInt16 CKilobotFormation::id_counter = 0;

CKilobotFormation::CKilobotFormation() :
   motors(NULL),
   leds(NULL),
   // ground_sensors(NULL),
   com_rx(NULL),
   com_tx(NULL),
   current_state(KILOBOT_STATE_STOP),
   previous_state(KILOBOT_STATE_STOP),
   mean_walk_duration(240),
   max_turning_steps(50),
   walking_steps(1),
   turning_steps(1),
   motor_L(0.0f),
   motor_R(0.0f)
{
   rng = CRandom::CreateRNG( "argos" );
   CKilobotFormation::SetIdNum(this);
}

/****************************************/
/* Se leen lor parametros de confinguración*/
/****************************************/

void CKilobotFormation::Init(TConfigurationNode& t_node) {
    motors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    leds = GetActuator<CCI_KilobotLEDActuator>("kilobot_led");
    // ground_sensors = GetSensor<CCI_GroundSensor>("ground");
    com_rx = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");
    com_tx = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");

    //leyendo del archivo de configuración
    TConfigurationNode experiment_conf = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "framework");
    experiment_conf = GetNode(experiment_conf,"experiment");
    GetNodeAttribute(experiment_conf, "ticks_per_second", ticks_per_second);
    max_turning_steps = 5 * ticks_per_second;

    //TODO comprobar parametros de configuración
    GetNodeAttributeOrDefault(t_node, "mean_walk_duration", mean_walk_duration, mean_walk_duration);
    mean_walk_duration *= ticks_per_second;


    Reset();
}

/****************************************/
/****************************************/
//TODO parece que se llama dos veces cuando se pulsa reset en la interfaz grafica. Es un bug?
void CKilobotFormation::Reset() {
    walking_steps = rng->Exponential(mean_walk_duration);

    current_state = KILOBOT_STATE_MOVING;
    previous_state = KILOBOT_STATE_MOVING;
    motor_L = motor_R = PIN_FORWARD;


    leds->SetColor(CColor::PURPLE);
}

/****************************************/
/****************************************/

void CKilobotFormation::ControlStep() {

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


void CKilobotFormation::SetIdNum(CKilobotFormation* robot){
    robot->id_num = id_counter ++;
}


REGISTER_CONTROLLER(CKilobotFormation, "kilobot_formation_controller")
