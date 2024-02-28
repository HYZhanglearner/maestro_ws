#ifndef HAND_DISPLAY_H
#define HAND_DISPLAY_H

#include <iostream>
#include <iterator>
#include <array>
#include <GL/glut.h>
#include <cmath>
#include <deque>

#define NUMBER_OF_SENSOR_THUMB 6
#define NUMBER_OF_SENSOR_FINGER 5
class HandDisplay {
private:
    // Variables to store the angles for each finger joint.
    double add_abdThumb, MCP_thumb, PIP_thumb, DIP_thumb;
    double add_abdIndex, MCP_index, PIP_index, DIP_index;
    double add_abdMid, MCP_Mid, PIP_Mid, DIP_Mid;

    double thumbAngles[NUMBER_OF_SENSOR_THUMB], indexAngles[NUMBER_OF_SENSOR_FINGER], middleAngles[NUMBER_OF_SENSOR_FINGER];
    static HandDisplay* instance;  // static pointer to an instance
    
public:
    static void displayWrapper();
    
    HandDisplay();
    ~HandDisplay();

    void initialize(int argc, char** argv);
    void displayHand();
    void updateFingerAngles(const std::array<double, NUMBER_OF_SENSOR_THUMB + 2*NUMBER_OF_SENSOR_FINGER>& sensorData);

    std::array<double, 16> fetchDataFromSensors();

    static void staticTimerCallback(int value);
    void timerCallback(int value);

    enum finger {thumb=0, index=1, middle=2};
    void drawFinger(double angles[], const std::array<float, 3>&  basePosi, 
                             float Radius, const std::array<float, 3>&  segmentLength,
                             const std::array<float, 3>& color, HandDisplay::finger fingerName);


    static void staticPlotTimerCallback(int value);

    std::array<double, 4> getThumbAngles();
    std::array<double, 4> getIndexAngles();
    std::array<double, 4> getMiddleAngles();

    void drawPalm();
    void drawLineSegment(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2);
    void updateCamera(); // rotate camera
    
};

// Global variables to hold the rotation angles
extern GLfloat cameraAngleX;
extern GLfloat cameraAngleY;
extern GLfloat cameraDistance;
extern GLfloat rotate_x;
extern GLfloat rotate_y;
extern GLfloat lastX, lastY;
extern bool firstMouse;

#endif // HAND_DISPLAY_H
