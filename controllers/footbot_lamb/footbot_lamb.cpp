/* Include the controller definition */
#include "footbot_lamb.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/
UInt8 CFootBotLamb::id_counter = 0;

CFootBotLamb::CFootBotLamb() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcPosSens(NULL),
    m_pcRBSens(NULL),
    m_pcRBAct(NULL),
    m_cAlpha(10.0f),
    m_fDelta(0.5f),
    m_fWheelVelocity(2.5f),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
    ping_interval(3),
    mess_count(0),
    clear_message(false),
    pos() {
        CFootBotLamb::SetIdNum(this);
}

/****************************************/
/****************************************/

void CFootBotLamb::Init(TConfigurationNode& t_node) {
   // Get sensor/actuator handles
   m_pcWheels      = GetActuator<CCI_DifferentialSteeringActuator >("differential_steering");
   m_pcProximity   = GetSensor  <CCI_FootBotProximitySensor       >("footbot_proximity"    );
   m_pcPosSens     = GetSensor  <CCI_PositioningSensor            >("positioning"          );
   m_pcRBAct       = GetActuator<CCI_RangeAndBearingActuator      >("range_and_bearing"    );
   m_pcRBSens      = GetSensor  <CCI_RangeAndBearingSensor        >("range_and_bearing"    );


   //leyendo del archivo de configuración
   TConfigurationNode experiment_conf = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "framework");
   experiment_conf = GetNode(experiment_conf,"experiment");
   UInt32 ticks_per_second;
   GetNodeAttribute(experiment_conf, "ticks_per_second", ticks_per_second);

   // Parse the configuration file
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   GetNodeAttributeOrDefault(t_node, "ping_interval", ping_interval, ping_interval);
   ping_interval *= ticks_per_second;
   Reset();

}

void CFootBotLamb::Reset() {
    mess_count = 0;
    clear_message = false;
    // m_pcRBAct->ClearData();
    ping_timer = ping_interval;
    neightbors.clear();
}

/****************************************/
/****************************************/

void CFootBotLamb::ControlStep() {
    // OBTENEMOS EL ID DEL ROBOT E IMPRIMIMOS
    string id = this->GetId();

    // LEEMOS EL SENSOR DE POSICION E IMPRIMIMOS
    pos = m_pcPosSens->GetReading().Position;
    if(ENABLE_DEBUG) LOG << "ID: " << id << "   x: " << pos[0] << "\t\ty: " << pos[1] << "\t\tz: " << pos[2] << endl;

    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i) {
    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
    CRadians cAngle = cAccumulator.Angle();

    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
    }

    //FIXME pone a 0 el mensaje, es un apaño,
    //pero no soy capaz de que deje de enviarse en cada step
    if(clear_message){
        m_pcRBAct->ClearData();
        clear_message = false;
    }

    if(--ping_timer <=0){
        Ping();
        ping_timer = ping_interval;
    }
    PollMessages();
}


void CFootBotLamb::Ping(){
    neightbors.clear();

    UInt8 *data_b = new UInt8[26];
    data_b[0] = id_num;
    data_b[1] = (UInt8) CODE_PING;

    m_pcRBAct->SetData(CByteArray(data_b, 26));
    delete data_b;
    clear_message = true; //para enviarlo una sola vez
}

void CFootBotLamb::SendPosition(){

    CByteArray mess_data;
    mess_data << id_num;
    mess_data << (UInt8) CODE_PING_REPLY;
    mess_data << pos.GetX();
    mess_data << pos.GetY();
    m_pcRBAct->SetData(mess_data);

    clear_message = true; //para enviarlo una sola vez
}


void CFootBotLamb::PollMessages(){
    CCI_RangeAndBearingSensor::TReadings messages = m_pcRBSens->GetReadings();
    CCI_RangeAndBearingSensor::TReadings::iterator m;
    for (m = messages.begin() ; m != messages.end(); ++m){
        switch (m->Data[1]) {
            case CODE_PING:
                SendPosition();
                break;
            case CODE_PING_REPLY:
                Neightbor_Info n = neightbors[m->Data.PopFront<UInt8>()];
                m->Data.PopFront<UInt8>(); //descartar el codigo del mensaje (Data[1])
                n.pos.SetX(m->Data.PopFront<Real>());
                n.pos.SetY(m->Data.PopFront<Real>());
                n.range = m->Range;
                n.bearing = m->HorizontalBearing;


                // LOG<<GetId()<<" recibe de "<<n.id<<", pos: ("<< n.pos.GetX()<<", " <<n.pos.GetY()<<"), range: " << n.range<< ", bearing: "<< n.bearing<<")\n";
                break;
        }
    }
    // if(messages.size() > 0)
    //     LOG<<GetId()<<" tiene "<<neightbors.size()<<" vecinos\n";
}


void CFootBotLamb::SetIdNum(CFootBotLamb* robot){
    robot->id_num = id_counter ++;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotLamb, "footbot_lamb_controller")
