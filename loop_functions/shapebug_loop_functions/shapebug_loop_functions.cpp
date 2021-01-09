#include "shapebug_loop_functions.h"
CShapebugLoopFunctions::CShapebugLoopFunctions():
 CLoopFunctions(){}

void CShapebugLoopFunctions::Init(TConfigurationNode& t_tree) {
    // lee los paramatros del archivo de configuracion con extesion .argos
    if(NodeExists(t_tree, "params")){
        TConfigurationNode params = GetNode(t_tree, "params");
        char * image_source;
        GetNodeAttribute(params, "image_source", image_source);
        shape = BMP1b(image_source);
        LOG<<image_source<<endl;
        if (shape.isLoaded())
            LOG<<"CARGADO TRON"<<endl;
        else
            LOG<<"QUE VA TIO"<<endl;
    }

}

/****************************************/
/****************************************/

void CShapebugLoopFunctions::Reset() {

}


CColor CShapebugLoopFunctions::GetFloorColor(const CVector2& pos){
    double x = pos.GetX() / 2.5;
    double y = pos.GetY() / 2.5;
    if(shape.getColor(1024*x, 1024*y))
        return CColor::WHITE;
    return CColor::BLACK;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CShapebugLoopFunctions, "shapebug_loop_functions")
