#pragma once
#ifdef USE_GUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <stdio.h>

class GSLIMGUI{
    public:
        static void setup();
        static void close();
        static void StartFrame();
        static void Render();
    
        static GLFWwindow* window;
};
#endif