#include "hand_display.h"
#include <GL/glut.h>


void mouseButton(int button, int state, int x, int y);
void mouseMotion(int x, int y);

GLfloat cameraAngleX = 0.5f; // 摄像机绕X轴旋转的角度
GLfloat cameraAngleY = 0.4f; // 摄像机绕Y轴旋转的角度
GLfloat cameraDistance = 2.5f; // 摄像机距离观察目标点的距离

GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat rotate_x = 0.0f;
GLfloat rotate_y = 0.0f;

HandDisplay handDisplay; //

int main(int argc, char** argv) {
    // Initialize HandDisplay and OpenGL
   
    handDisplay.initialize(argc, argv);
   
    glutMouseFunc(mouseButton);  // Register mouse button callback
    glutMotionFunc(mouseMotion);  // Register mouse motion callback

    // Begin the GLUT main loop. This will continuously call the registered display function.
    glutMainLoop();
    
    return 0;
}

void mouseButton(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        firstMouse = true;
    }
}


void mouseMotion(int x, int y) {
    if (firstMouse) {
        lastX = x;
        lastY = y;
        firstMouse = false;
    }

    GLfloat xoffset = x - lastX;
    GLfloat yoffset = lastY - y;
    lastX = x;
    lastY = y;

    GLfloat sensitivity = 1.0f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    rotate_x += yoffset;
    rotate_y += xoffset;

    cameraAngleX = yoffset; // 这里可能需要一些调整来匹配旋转的方向和速度
    cameraAngleY = xoffset; // 同上

    handDisplay.updateCamera(); // 更新摄像机
    glutPostRedisplay();
}

