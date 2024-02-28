#include "hand_display.h"
#include <GL/glut.h>
#include <cstdlib>  // for rand() function

// Initialize the static instance pointer
HandDisplay* HandDisplay::instance = nullptr;


HandDisplay::HandDisplay() {
    GLfloat light_pos[] = {1.0, 1.0, 1.0, 0.0};
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glEnable(GL_DEPTH_TEST);
    instance = this;  // set the static instance pointer in the constructor

    // Initialize angles or other data.
    thumbAngles[0] = 0.0f; thumbAngles[1] = 0.0f; thumbAngles[2] = 0.0f; thumbAngles[3] = 0.0f;
    indexAngles[0] = 0.0f; indexAngles[1] = 0.0f; indexAngles[2] = 0.0f;
    middleAngles[0] = 0.0f; middleAngles[1] = 0.0f; middleAngles[2] = 0.0f;
}

HandDisplay::~HandDisplay() {
    instance = nullptr;
}

void HandDisplay::initialize(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Robot Hand Display");
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(60.0, 800.0/600.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(-0.0, 3, 3, 0, 0, 0, 0, 1, 0); // Simple camera setup looking at the origin

    // Set up other OpenGL options, like lighting.

    glutDisplayFunc(displayWrapper); // Register the static display wrapper function.


    // glutTimerFunc(10, HandDisplay::staticTimerCallback, 0);  // Call timerCallback every 10 milliseconds

    // glutTimerFunc(100, &HandDisplay::staticPlotTimerCallback, 0);  // 
}

void HandDisplay::updateFingerAngles(const std::array<double, 16>& sensorData) {
    for(int ii = 0; ii < NUMBER_OF_SENSOR_THUMB; ii++)
    {   
        double gain = 1.0;
        // TO-DO apply psuedo inverse model
        thumbAngles[ii] = gain * sensorData[ii] * (180.0 / M_PI);   // Taking the 4th value from the thumb sensor data

    }

    for(int ii = 0; ii < NUMBER_OF_SENSOR_FINGER; ii++){
        double gain = 1.0;
        // TO-DO apply psuedo inverse model
        indexAngles[ii] = gain * sensorData[NUMBER_OF_SENSOR_THUMB + ii] * (180.0 / M_PI);   // Taking the 4th value from the index finger sensor data

    }
    
    for(int ii = 0; ii < NUMBER_OF_SENSOR_FINGER; ii++){
        double gain = 1.0;
        // TO-DO apply psuedo inverse model
        middleAngles[ii] = sensorData[NUMBER_OF_SENSOR_THUMB + NUMBER_OF_SENSOR_FINGER + ii] * (180.0 / M_PI);   // Taking the 4th value from the middle finger sensor data
    }

    // // Debug print
    // std::cout << "Thumb Angles: " << std::endl;
    // std::cout << "Index Angles: " << indexAngles[0] << ", " << indexAngles[1] << ", " << indexAngles[2] << std::endl;
    // std::cout << "Middle Angles: " << middleAngles[0] << ", " << middleAngles[1] << ", " << middleAngles[2] << std::endl;

    glutPostRedisplay();  // Request to redraw the screen.
    
}

void HandDisplay::displayHand() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // glClearColor(0.7f, 0.7f, 0.7f, 1.0f);

    // 设置投影矩阵
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // gluPerspective(60.0, 800.0/600.0, 0.1, 100.0);

    // 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    updateCamera(); // 调用更新摄像机的函数

    

    // drawing plam
    drawPalm();

    // Draw the thumb
    
    drawFinger(thumbAngles, {0.0f, 0.0f, -0.7f}, 0.03, {0.45f, 0.35f, 0.28f}, {1.0f, 0.0f, 0.0f}, thumb);

    // Draw the index finger
    drawFinger(indexAngles, {0.0, -0.0f, 0}, 0.03, {0.45f, 0.27f, 0.18f}, {0.0f, 1.0f, 0.0f}, index);

    // Draw the middle finger
    drawFinger(middleAngles, {0.3, -0.0f, 0}, 0.03, {0.53f, 0.33f, 0.18f}, {0.0f, 0.0f, 1.0f}, middle);

    glutSwapBuffers();  // Swap buffers for double buffering.
}

void HandDisplay::drawPalm(){
    glPushMatrix();  // 保存当前矩阵

    GLfloat pt1[3] = {0.0f, 0.0f, -0.7f},
    pt2[3] = {0.0f, -0.0f, 0.0f},
    pt3[3] = {0.3f, -0.0f, 0.02f},
    pt4[3] = {0.6f, -0.0f, 0.0f},
    pt5[3] = {0.85f, -0.0f, -0.02f},
    pt6[3] = {0.75f, -0.0f, -0.75f},
    pt7[3] = {0.3f, -0.0f, -0.85f};

    drawLineSegment(pt1[0], pt1[1], pt1[2], pt2[0], pt2[1], pt2[2]);
    drawLineSegment(pt2[0], pt2[1], pt2[2], pt3[0], pt3[1], pt3[2]);
    drawLineSegment(pt3[0], pt3[1], pt3[2], pt4[0], pt4[1], pt4[2]);
    drawLineSegment(pt5[0], pt5[1], pt5[2], pt4[0], pt4[1], pt4[2]);
    drawLineSegment(pt5[0], pt5[1], pt5[2], pt6[0], pt6[1], pt6[2]);
    drawLineSegment(pt7[0], pt7[1], pt7[2], pt6[0], pt6[1], pt6[2]);
    drawLineSegment(pt7[0], pt7[1], pt7[2], pt1[0], pt1[1], pt1[2]);
    
    drawLineSegment(pt2[0], pt2[1], pt2[2], pt7[0], pt7[1], pt7[2]);
    drawLineSegment(pt3[0], pt3[1], pt3[2], pt7[0], pt7[1], pt7[2]);
    drawLineSegment(pt4[0], pt4[1], pt4[2], pt7[0], pt7[1], pt7[2]);
    drawLineSegment(pt5[0], pt5[1], pt5[2], pt7[0], pt7[1], pt7[2]);
    glPopMatrix();  // 恢复之前保存的矩阵

}


void HandDisplay::displayWrapper() {
    if(instance) {
        instance->displayHand();
    }
}

void HandDisplay::timerCallback(int value) {
    // Fetch and process sensor data
    // std::array<double, NUMBER_OF_SENSOR_THUMB + 2*NUMBER_OF_SENSOR_FINGER> sensorData = fetchDataFromSensors();  // You need to implement this function or equivalent to fetch data
    // updateFingerAngles(sensorData);

    std::cout << "Thumb Angles: " << thumbAngles[0] << ", " << thumbAngles[1] << ", " << thumbAngles[2] << std::endl;
    std::cout << "Index Angles: " << indexAngles[0] << ", " << indexAngles[1] << ", " << indexAngles[2] << std::endl;
    std::cout << "Middle Angles: " << middleAngles[0] << ", " << middleAngles[1] << ", " << middleAngles[2] << std::endl;

    glutTimerFunc(10, HandDisplay::staticTimerCallback, value);  // Reset the timer for another 10 milliseconds
}

std::array<double, 16> HandDisplay::fetchDataFromSensors() {
    std::array<double, 16> data;

    // Fetch the data from your sensors here
    // For demonstration purposes, I'll simulate it
    for (int i = 0; i < 16; ++i) {
        data[i] = 0.0; // fixed value
        // data[i] = (std::rand() % 360) * (M_PI / 180); 
    }
        
    return data;
}

void HandDisplay::staticTimerCallback(int value) {
    if (instance) {
        instance->timerCallback(value);
    }
}

void HandDisplay::drawFinger(double angles[], const std::array<float, 3>&  basePosi, 
                             float Radius, const std::array<float, 3>&  segmentLength,
                             const std::array<float, 3>& color, finger finger) {
    
    GLUquadric* quad = gluNewQuadric();

    glPushMatrix();
    glTranslatef(basePosi[0], basePosi[1], basePosi[2]);
    
    switch(finger){ 
    case thumb:{
        glRotatef(-60.0, 0.0, 1.0, 0.0);
        
        // Abduction/Adduction
        add_abdThumb = angles[1] + 50;
        glRotatef(add_abdThumb, 1.0, 0.0, 0.0);

        // First segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        MCP_thumb = 1.0*(angles[0]);
        glRotatef(MCP_thumb, 0.0, 1.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[0], 20, 5);
        
        // Move to the end of the first segment
        glTranslatef(0.0, 0.0, segmentLength[0]);

        // Second segment
        // Set Color
        PIP_thumb = 2.5*(angles[2]);
        // PIP_thumb = 0.902671*57.29;

        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        glRotatef(PIP_thumb, 0.0, 1.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[1], 20, 5);
        
        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[1]);

        // Third segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        DIP_thumb = 1.0*(angles[4]);
        glRotatef(DIP_thumb, 0.0, 1.0, 0.0); // DIP is driven
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[2], 20, 5);

        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[2]);
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        break;
    }
    case index:{
        // Abduction/Adduction
        add_abdIndex = angles[1];
        glRotatef(add_abdIndex, 0.0, 1.0, 0.0);

        // First segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        MCP_index = 2.5 * angles[0];
        glRotatef(MCP_index, 1.0, 0.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[0], 20, 5);
        
        // Move to the end of the first segment
        glTranslatef(0.0, 0.0, segmentLength[0]);

        // Second segment
        // Set Color
        PIP_index = 0.7*(angles[2]);

        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        glRotatef(PIP_index, 1.0, 0.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[1], 20, 5);
        
        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[1]);

        // Third segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint

        DIP_index = 0.66*PIP_index;
        glRotatef(DIP_index, 1.0, 0.0, 0.0); // DIP is driven
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[2], 20, 5);

        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[2]);
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        break;
    } 
    case middle:{
        glRotatef(15, 0.0, 1.0, 0.0);
        // Abduction/Adduction
        add_abdMid = angles[1];
        glRotatef(add_abdMid, 0.0, 1.0, 0.0);

        // First segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        MCP_Mid = 2.5* angles[0];
        glRotatef(MCP_Mid, 1.0, 0.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[0], 20, 5);
        
        // Move to the end of the first segment
        glTranslatef(0.0, 0.0, segmentLength[0]);

        // Second segment
        // Set Color
        PIP_Mid = 1.0* (angles[2]);

        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        glRotatef(PIP_Mid, 1.0, 0.0, 0.0);
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[1], 20, 5);
        
        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[1]);

        // Third segment
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint

        DIP_Mid = 0.66*PIP_Mid;
        glRotatef(DIP_Mid, 1.0, 0.0, 0.0); // DIP is driven
        // Set Color
        glColor3f(color[0], color[1], color[2]);
        gluCylinder(quad, Radius, Radius, segmentLength[2], 20, 5);

        // Move to the end of the second segment
        glTranslatef(0.0, 0.0, segmentLength[2]);
        // Set Color
        glColor3f(0.5f, 0.5f, 0.5f);
        glutSolidSphere(0.05, 20, 20); // Joint
        break;
    }
    }
    glPopMatrix();
}

std::array<double, 4> HandDisplay::getThumbAngles() {
    return { add_abdThumb, MCP_thumb, PIP_thumb, DIP_thumb };
}

std::array<double, 4> HandDisplay::getIndexAngles() {
    return { add_abdIndex, MCP_index, PIP_index, DIP_index };
}

std::array<double, 4> HandDisplay::getMiddleAngles() {
    return { add_abdMid, MCP_Mid, PIP_Mid, DIP_Mid };
}

void HandDisplay::staticPlotTimerCallback(int value) {
    std::cout << "Timer callback called!" << std::endl;
    if (instance) {

    }
}

void HandDisplay::drawLineSegment(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2) {
    glPushMatrix();

    glLineWidth(5.0f);  // Set the line thickness

    glBegin(GL_LINES);  // Start drawing lines
        glColor3f(0.8f, 0.8f, 0.8f);  // Set color (optional)
        glVertex3f(x1, y1, z1);  // First endpoint of the line
        glVertex3f(x2, y2, z2);  // Second endpoint of the line
    glEnd();  // End drawing lines
    
    // Set the size of the point
    glPointSize(10.0f);  // Replace 5.0f with the desired thickness

    // Begin drawing points
    glBegin(GL_POINTS);
        glVertex3f(x1, y1, z1);  // Specify the position of the point
    glEnd();

        // Begin drawing points
    glBegin(GL_POINTS);
        glVertex3f(x2, y2, z2);  // Specify the position of the point
    glEnd();

    glPopMatrix();
}

void HandDisplay::updateCamera() {
    // 计算摄像机的位置
    GLfloat camX = sin(cameraAngleY) * cos(cameraAngleX) * cameraDistance;
    GLfloat camY = sin(cameraAngleX) * cameraDistance;
    GLfloat camZ = cos(cameraAngleY) * cos(cameraAngleX) * cameraDistance;

    // 更新视点
    gluLookAt(camX, camY, camZ,  // 摄像机在世界坐标的位置
              0.0, 0.0, 0.0,     // 摄像机正看着的点
              0.0, 1.0, 0.0);    // 上方向量
}