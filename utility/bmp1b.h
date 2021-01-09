#ifndef BMP1b_H
#define BMP1b_H

#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>

using namespace std;


class BMP1b{
public:
    inline BMP1b(){loaded = false;}
    BMP1b(char *source);
    ~BMP1b();
    void setSource(char *source);
    unsigned char getColor(uint32_t x, uint32_t y);
    inline bool isLoaded(){return loaded;}

private:
    char *source;
    bool loaded;
    float aspect_ratio;
    uint32_t width, height;
    unsigned char * pixel_data;

    char *error_message;
};

#endif
