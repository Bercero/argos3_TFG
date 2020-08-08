#ifndef BAYESIAN_DECISION_LOOP_FUNCTIONS_H
#define BAYESIAN_DECISION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>//TODO puedo omitir el import?
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include "kilobot_bayesiandecision.h"

using namespace argos;

class CBayesianDecisionLoopFunctions : public CLoopFunctions {

public:
    CBayesianDecisionLoopFunctions();

    virtual ~CBayesianDecisionLoopFunctions() {}
    //se llama al comienzo de la simulacion y le los parametros del archivo de configuración
    void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    bool IsExperimentFinished();
    //convierte el suelo en una cuadricula y rellena cada cuardro con blanco o negro aleatoriamente
    //en una proporcion dada por el parametro "fill_ratio"
    virtual CColor GetFloorColor(const CVector2& c_pos_on_floor);

private:
    //parametros pasados en el archivo de configuración
    UInt32 grid_resolution_x;//numero de cuadrados en el eje x
    Real fill_ratio;

    Real square_size;//tamaño de los cuadros de la cuadricula
    CRandom::CRNG* rng;
    std::vector<UInt32> grid;//lista de casillas blancas

    //para calcular las cordenadas del suelo partiendo desde una esquina en lugar
    // de desde el centro
    Real offset_x, offset_y;

    std::vector<CKilobotBayesianDecision*> controllers;
};

#endif
