#include "bayesian_decision_loop_functions.h"

CBayesianDecisionLoopFunctions::CBayesianDecisionLoopFunctions(): CLoopFunctions(), fill_ratio(0.8), grid_resolution_x(10){}

void CBayesianDecisionLoopFunctions::Init(TConfigurationNode& t_tree) {

    //lee los paramatros del archivo de configuracion con extesion .argos
    if(NodeExists(t_tree, "params")){
        TConfigurationNode params = GetNode(t_tree, "params");
        GetNodeAttributeOrDefault(params, "fill_ratio", fill_ratio, fill_ratio);
        GetNodeAttributeOrDefault(params, "grid_resolution", grid_resolution_x, grid_resolution_x);
    }

    //obteniendo el generador de numeros aleatorios
    rng = CRandom::CreateRNG("argos");

    //obtiene las medidas del suelo
    CVector3 arena_size = CSimulator::GetInstance().GetSpace().GetArenaSize();
    //este desplazamiento es para calcular las cordenadas del suelo desde una esquina
    //a partir de las cordenadas con origen en el centro
    offset_x = arena_size.GetX()/2;
    offset_y = arena_size.GetY()/2;

    //el tamaño del cuadro depende del número de cuadros en el eje x
    square_size = arena_size.GetX()/grid_resolution_x;
    //calculo del numero de cuadros del eje y a partir del tamaño del cuadro
    UInt32 grid_resolution_y = arena_size.GetY()/square_size;

    //si el número de cuadros de alguna dimension no es entero se amplia el número en uno
    if (square_size*grid_resolution_x < arena_size.GetX())
        grid_resolution_x ++ ;
    if (square_size*grid_resolution_y < arena_size.GetY())
        grid_resolution_y ++ ;

    //vector con los indices de todos los cuadros
    for(int i=0; i< (grid_resolution_x * grid_resolution_y); i++){
        grid.push_back(i);
    }
    //se eliminan indices aletorimente hasta que quedan fill_ratio*numero_inicial de casillas
    UInt32 index=0;
    UInt32 target_size = fill_ratio * grid.size();
    while(grid.size() > target_size){
        index = rng->Uniform(CRange<UInt32>(0, grid.size()));
        grid.erase(grid.begin()+index);
    }

}

/****************************************/
/****************************************/

void CBayesianDecisionLoopFunctions::Reset() {
    rng->Reset();

}

CColor CBayesianDecisionLoopFunctions::GetFloorColor(const CVector2& c_pos_on_floor){
    Real pos_y = c_pos_on_floor.GetY() + offset_y;
    Real pos_x = c_pos_on_floor.GetX() + offset_x;
    UInt32 index = UInt32(pos_y/square_size) * (grid_resolution_x);
    index += UInt32(pos_x/square_size);
    for(int i = 0; i < grid.size() ; i++){
        if (index == grid.at(i))
            return CColor::WHITE;
    }
    return CColor::BLACK;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBayesianDecisionLoopFunctions, "bayesian_decision_loop_functions")
