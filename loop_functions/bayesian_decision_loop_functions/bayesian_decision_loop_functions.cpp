#include "bayesian_decision_loop_functions.h"
CBayesianDecisionLoopFunctions::CBayesianDecisionLoopFunctions():
 CLoopFunctions(), fill_ratio(0.8), grid_resolution_x(10), margin(5), limit_x(0), limit_y(0){}

void CBayesianDecisionLoopFunctions::Init(TConfigurationNode& t_tree) {
    //lee los paramatros del archivo de configuracion con extesion .argos
    if(NodeExists(t_tree, "params")){
        TConfigurationNode params = GetNode(t_tree, "params");
        //TODO comprobar valores de los parametros antes de aceptarlos
        GetNodeAttributeOrDefault(params, "fill_ratio", fill_ratio, fill_ratio);
        GetNodeAttributeOrDefault(params, "grid_resolution", grid_resolution_x, grid_resolution_x);
        GetNodeAttributeOrDefault(params, "margin", margin, margin);
        margin = margin / 100;
    }

    //obteniendo el generador de numeros aleatorios
    rng = CRandom::CreateRNG("argos");

    //obtiene las medidas del suelo
    CVector3 arena_size = CSimulator::GetInstance().GetSpace().GetArenaSize();
    //este desplazamiento es para calcular las cordenadas del suelo desde una esquina
    //a partir de las cordenadas con origen en el centro
    offset_x = (arena_size.GetX() / 2) - margin;
    offset_y = (arena_size.GetY() / 2) - margin;
    limit_x = arena_size.GetX() - 2*margin;
    limit_y = arena_size.GetY() - 2*margin;

    //el tamaño del cuadro depende del número de cuadros en el eje x
    square_size = limit_x / grid_resolution_x;
    //calculo del numero de cuadros del eje y a partir del tamaño del cuadro
    UInt32 grid_resolution_y = limit_y / square_size;

    //si el número de cuadros de alguna dimension no es entero se amplia el número en uno
    if (square_size*grid_resolution_x < limit_x)
        grid_resolution_x ++ ;
    if (square_size*grid_resolution_y < limit_y)
        grid_resolution_y ++ ;

    //vector con los indices de todos los cuadros
    for(UInt32 i=0; i< (grid_resolution_x * grid_resolution_y); i++){
        grid.push_back(i);
    }
    //se eliminan indices aletorimente hasta que quedan fill_ratio*numero_inicial de casillas
    UInt32 index=0;
    UInt32 target_size = fill_ratio * grid.size();
    while(grid.size() > target_size){
        index = rng->Uniform(CRange<UInt32>(0, grid.size()));
        grid.erase(grid.begin()+index);
    }
    //TODO comentar, tiene que ver con la condicion de salida
    CSpace::TMapPerType robot_map = CSimulator::GetInstance().GetSpace().GetEntitiesByType("kilobot");
    CKilobotEntity *kb;
    for (typeof(robot_map.begin()) it=robot_map.begin(); it!=robot_map.end(); ++it){
        kb = any_cast<CKilobotEntity*>(it->second);
        CCI_Controller *controller = &(kb->GetControllableEntity().GetController());
        controllers.push_back( dynamic_cast<CKilobotBayesianDecision*>(controller) );
    }
}

/****************************************/
/****************************************/

void CBayesianDecisionLoopFunctions::Reset() {
    // rng->Reset();//TODO Parece que no es necesario
    //TODO Tampoco parece necesario regenerar el vector de robots entre reinicios
    LOG<<"f = "<<fill_ratio<<"\n";
}

bool CBayesianDecisionLoopFunctions::IsExperimentFinished() {
    w_decision = 0;
    for(UInt32 i = 0; i < controllers.size(); i++){
        decision_i = controllers[i]->GetDecision();
        if( decision_i == -1)
            return false;
        w_decision += decision_i;
    }
    LOG<<"Experimento finalizado: Todos los robots han tomado una decision\n";
    b_decision = controllers.size() - w_decision;
    LOG<<"f = "<<fill_ratio<<"\n";
    LOG<<"decision w = "<<w_decision<<"\n";
    LOG<<"decision b = "<<b_decision<<"\n";

    return true;
}

CColor CBayesianDecisionLoopFunctions::GetFloorColor(const CVector2& c_pos_on_floor){
    Real pos_y = c_pos_on_floor.GetY() + offset_y;
    Real pos_x = c_pos_on_floor.GetX() + offset_x;
    if(pos_y < 0 || pos_x <0 || pos_y > limit_y || pos_x > limit_x){
        return CColor::GRAY50;
    }
    UInt32 index = UInt32(pos_y/square_size) * (grid_resolution_x);
    index += UInt32(pos_x/square_size);
    for(UInt32 i = 0; i < grid.size() ; i++){
        if (index == grid.at(i))
            return CColor::WHITE;
    }
    return CColor::BLACK;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBayesianDecisionLoopFunctions, "bayesian_decision_loop_functions")
