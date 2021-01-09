#ifndef SHAPEBUG_LOOP_FUNCTIONS_H
#define SHAPEBUG_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "utility/bmp1b.h"
// #include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_entity.h>

using namespace argos;

class CShapebugLoopFunctions : public CLoopFunctions {

public:
    CShapebugLoopFunctions();

    virtual ~CShapebugLoopFunctions() {}

    void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual CColor GetFloorColor(const CVector2& pos);

private:

BMP1b shape;
};

#endif
