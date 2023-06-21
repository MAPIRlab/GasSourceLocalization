#pragma once
#include <implot/implot.h>

class GSLIMPLOT{
    public:
    //call right after gslimgui.setup
    static void setup(){
        ImPlot::CreateContext();
    }
    //call right before gslimgui.close
    static void close(){
        ImPlot::DestroyContext();
    }
    
};