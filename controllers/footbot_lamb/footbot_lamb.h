#ifndef FOOTBOT_LAMB_H
#define FOOTBOT_LAMB_H

/*
 * Include some necessary headers.
 */
#include <iostream>
#include <cstring>
#include <map>

// core for log
 #include <argos3/core/simulator/simulator.h>
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/*radio sensor and actuartor*/
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

// id obtaining and entity workflow
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>


#define CODE_PING 1
#define CODE_PING_REPLY 2
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;
/*
 * A controller is simply an implementation of the CCI_Controller class.
 */

class CFootBotLamb : public CCI_Controller {

public:

    struct Neightbor_Info{
        CVector2 pos;
        Real range;
        CRadians bearing;
    };

    /* Class constructor. */
    CFootBotLamb();

    /* Class destructor. */
    virtual ~CFootBotLamb() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    void Ping();
    void SendPosition();
    void PollMessages();

    // activa o desactiva los prints
    bool ENABLE_DEBUG = false;

private:
    void static SetIdNum(CFootBotLamb* robot);

    static UInt8 id_counter;
    UInt8 id_num;

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor* m_pcProximity;
    // Pointer to pos sensor
    CCI_PositioningSensor* m_pcPosSens;
    //Radio sensor and actuators
    CCI_RangeAndBearingSensor *m_pcRBSens;
    CCI_RangeAndBearingActuator *m_pcRBAct;
    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

    /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
    CDegrees m_cAlpha;
    /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
    Real m_fDelta;
    /* Wheel speed. */
    Real m_fWheelVelocity;
    /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
    CRange<CRadians> m_cGoStraightAngleRange;

    Real ping_interval;
    Real ping_timer;
    UInt8 mess_count;
    bool clear_message;

    CVector3 pos;
    std::map<UInt8,Neightbor_Info> neightbors;


};

#endif
