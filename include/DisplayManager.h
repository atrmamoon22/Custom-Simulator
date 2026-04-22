#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>

class DisplayManager {
    public:
        void createDisplay();
        void updateDisplay();
        bool shouldClose();
        void closeDisplay();
        
        double getFrameTime();
        
        static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
        void processInput(GLFWwindow *window);
        GLFWwindow* getWindow() const;
        const int getWidth();
        const int getHeight();
        
    private:
        double getCurrentTime();

        const int WINDOW_WIDTH = 640;
        const int WINDOW_HEIGHT = 480;
        const std::string TITLE = "Simulator";
        double lastFrameTime;
        double delta = 0.0;
        GLFWwindow* window;
};
#endif