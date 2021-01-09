#include "bmp1b.h"

BMP1b::BMP1b(char *source):BMP1b(){
    setSource(source);
}

BMP1b::~BMP1b(){
    if (loaded) delete pixel_data;
}


void BMP1b::setSource(char * source){
    // liberando la memoria si se hubiese cargado otra imagen antes
    if (loaded) delete pixel_data;
    loaded = false;

    ifstream file(source, ios::binary);
    //TODO controlar los errores un poco mejor
    if(!file) return;

    this->source = source;

    //el header tiene por lo menos 54 bytes aunque puden ser más
    char * header = new char[54];
	file.read(header, 54);

    //La unica comprobacion que hago es que la del número magico del archivo
    if (header[0] == 'B' && header[1] == 'M'){
        uint32_t data_begin = *(uint32_t *) (header+10);
                      width = *(uint32_t *) (header+18);
                     height = *(uint32_t *) (header+22);
        uint32_t image_size = *(uint32_t *) (header+34);

        pixel_data = new unsigned char [image_size];
        file.seekg(data_begin);
        file >> pixel_data;
        loaded = true;

        header = NULL;  delete header;
    }
}



unsigned char BMP1b::getColor(uint32_t x, uint32_t y){
    if (loaded){
        uint32_t pixel = (y * width) + x;
        uint32_t byte = pixel / 8;
        unsigned char offset = 1 << (unsigned char) (pixel - 8*byte);//pixel%8
        return offset & pixel_data[byte] ;
    }
    return 0;
}
