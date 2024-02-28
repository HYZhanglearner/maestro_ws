#include <iostream>
#include <thread>
#include <chrono>
#include <array>          // For using std::array
#include <system_error>   // For handling system errors
#include "UDP_receiver.h"
#include <sys/timerfd.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <iterator>
#include <hand_display.h>
#include <signal.h>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define NUMBER_OF_SENSOR_THUMB 6
#define NUMBER_OF_SENSOR_FINGER 5

GLfloat cameraAngleX = 0.0f; // 摄像机绕X轴旋转的角度
GLfloat cameraAngleY = 0.0f; // 摄像机绕Y轴旋转的角度
GLfloat cameraDistance = 3.0f; // 摄像机距离观察目标点的距离

GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat rotate_x = 0.0f;
GLfloat rotate_y = 0.0f;

std::atomic<bool> stopCommThread(false);
auto previous_time_1 = std::chrono::high_resolution_clock::now();
auto previous_time_2 = std::chrono::high_resolution_clock::now();

int setup_timer_fd_for_100Hz();
void communicationThread(int argc, char **argv, UdpCommunicator& slaveDevice);
void signalHandler(int signum);
void idleFunction();
// display
std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER> received_data;
std::mutex data_mutex;  // Mutex to protect received_data
std::condition_variable data_cv;  // Condition variable to notify of new data

HandDisplay handDisplay;
// Camera motion function
void mouseButton(int button, int state, int x, int y);
void mouseMotion(int x, int y);

// Finger configure transfer to ROS
std::vector<double> fingerJoints;

// ExoToHandKinematic
std::vector<double> ExoToHandKinematic(std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER>
 received_data);

// Define global ROS 2 node and publisher
rclcpp::Node::SharedPtr node;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    // UDP communication
    unsigned int port = 8080;
    UdpCommunicator slaveDevice(port);

    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("hand_jointSpace_node");

    // Create a publisher for a std_msgs/String message
    publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("hand_joint_space", 10);

    // Start the communication in a separate thread
    std::thread commThread(communicationThread, argc, argv, std::ref(slaveDevice));
    
    // Initialize HandDisplay and OpenGL
    
    handDisplay.initialize(argc, argv);

    glutMouseFunc(mouseButton);  // Register mouse button callback
    glutMotionFunc(mouseMotion);  // Register mouse motion callback

    // Set the idle callback
    glutIdleFunc(idleFunction);

    // Start the GLUT event processing loop
    glutMainLoop();


    commThread.join();
    slaveDevice.close();
    return 0;
}

void communicationThread(int argc, char **argv, UdpCommunicator& slaveDevice)  {    
    // number of joints
    int num_joints = 12;

    std::array<double, 16> data_to_send = {
        1.23, 3.45, 4.326, 5.672,
        1.672, 3.45, 4.23, 5.326,
        1.672, 3.326, 4.45, 5.23,
        1.672, 3.326, 4.45, 5.23,
    };
    udp::endpoint sender_endpoint;

    int timer_fd = setup_timer_fd_for_100Hz();
    int udp_fd = slaveDevice.getSocketFd();  

    fd_set read_fds;

    while (!stopCommThread.load()) { // check the flag
        
        // ... The select and FD_ISSET based communication code goes here ...
        FD_ZERO(&read_fds);
        FD_SET(timer_fd, &read_fds);
        FD_SET(udp_fd, &read_fds);

        int ret = select(std::max(timer_fd, udp_fd) + 1, &read_fds, NULL, NULL, NULL);

        if (ret > 0) {
            if (FD_ISSET(timer_fd, &read_fds)) {
                // Check the frequency
                auto current_time = std::chrono::high_resolution_clock::now();
                double interval = std::chrono::duration<double>(current_time - previous_time_1).count();
                double frequency = 1.0 / interval;
                std::cout << "Sending frequency: " << frequency << " Hz" << std::endl;
                previous_time_1 = current_time;
                        
                // Sending data
                uint64_t expirations;
                read(timer_fd, &expirations, sizeof(expirations));
                
                
                // Embed current timestamp into the data before sending
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                data_to_send[15] = static_cast<double>(millis); // Store timestamp as the last entry

                // Here, you send your data at 100Hz rate
                slaveDevice.send(data_to_send, sender_endpoint);
            }

            // receiving data
            if (FD_ISSET(udp_fd, &read_fds)) {
                try {
                    // Check the frequency        
                    auto current_time = std::chrono::high_resolution_clock::now();
                    double interval = std::chrono::duration<double>(current_time - previous_time_2).count();
                    double frequency = 1.0 / interval;
                    std::cout << "Receiving frequency: " << frequency << " Hz" << std::endl;
                    previous_time_2 = current_time;

                    // Receiving data
                    std::lock_guard<std::mutex> lock(data_mutex);
                    received_data = slaveDevice.receive(sender_endpoint);
                    // received_data[4] += 2.85;
                    fingerJoints = ExoToHandKinematic(received_data);

                    // publish to topic
                    std_msgs::msg::Float64MultiArray finger_joints_msg;
                    finger_joints_msg.data = fingerJoints;
                    publisher->publish(finger_joints_msg);

                    // Notify the main loop of new data
                    data_cv.notify_one();

                }
                catch(const std::system_error& e) {
                    std::cerr << "Caught system error: " << e.what() << '\n';
                }
                catch(const std::exception& e) {
                    std::cerr << "Caught exception: " << e.what() << '\n';
                }
                catch(...) {
                    std::cerr << "Caught unknown exception\n";
                }
            }
        }
        else if (ret < 0){
            
            perror("select");  // Print a descriptive error message
            break;
        }
        // If needed, add a termination condition to exit this loop
        
    }

    close(udp_fd);
    close(timer_fd);
}

int setup_timer_fd_for_100Hz(){

    int fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (fd == -1) {
        std::cerr << "Failed to create timer fd." << std::endl;
        exit(1);
    }

    struct itimerspec new_timer;
    memset(&new_timer, 0, sizeof(new_timer));
    new_timer.it_value.tv_sec = 0;
    new_timer.it_value.tv_nsec = 10 * 1000000; // Start in 10ms
    new_timer.it_interval.tv_sec = 0;
    new_timer.it_interval.tv_nsec = 10 * 1000000; // Repeat every 10ms

    if (timerfd_settime(fd, 0, &new_timer, NULL) == -1) {
        std::cerr << "Failed to start timer." << std::endl;
        exit(1);
    }

    return fd;
}

void signalHandler(int signum) {
    stopCommThread.store(true);
}

// Additional helper function to reset the hand display to default
void resetHandDisplay(HandDisplay& handDisplay) {
    std::array<double, NUMBER_OF_SENSOR_THUMB + 2 * NUMBER_OF_SENSOR_FINGER> default_data = {0}; // All zeros for example
    handDisplay.updateFingerAngles(default_data);
    handDisplay.displayHand();
}

// A function to print the received data to the console
void printReceivedData(const std::array<double, NUMBER_OF_SENSOR_THUMB + 2 * NUMBER_OF_SENSOR_FINGER>& data) {
    std::cout << "Received data: ";
    for(const auto& val : data) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

// A function to visually indicate a connection error on the hand display (e.g., by showing the hand in red)
void indicateErrorOnHandDisplay(HandDisplay& handDisplay) {
    handDisplay.displayHand();
}

void idleFunction() {

    // This is the equivalent of the while loop's body in your main() function

    // Wait for new data
    std::unique_lock<std::mutex> lock(data_mutex);
    if(data_cv.wait_for(lock, std::chrono::milliseconds(10)) == std::cv_status::timeout && stopCommThread.load())
    {
        exit(0);  // or another way to safely end the program
    }
    
    // Update hand representation
    handDisplay.updateFingerAngles(received_data);
    // You don't explicitly call handDisplay.displayHand() here 
    // because the GLUT display callback should handle rendering.
    // But you can request GLUT to render the display using:
    glutPostRedisplay();

    if (stopCommThread.load()) {
        exit(0);  // or another way to safely end the program
    }
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

// Forward Kinematic between Mastro and hand
std::vector<double> ExoToHandKinematic(std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER>
 received_data){
    // Thumb the angles are {add/abd MCP PIP DIP}
    std::array<double, 4> thumbToSend;
    std::array<double, 4> indexToSend;
    std::array<double, 4> middleToSend;
    thumbToSend = {received_data[0] -60 *M_PI/180, received_data[1] +50.0 *M_PI/180,
                     received_data[2], received_data[4]};

    indexToSend = {2.5 *received_data[NUMBER_OF_SENSOR_THUMB], received_data[NUMBER_OF_SENSOR_THUMB+1],
                    0.7 *received_data[NUMBER_OF_SENSOR_THUMB+2],
                    0.66 *0.7 *received_data[NUMBER_OF_SENSOR_THUMB+2]};
                    
    middleToSend = {2.5 *received_data[NUMBER_OF_SENSOR_THUMB+NUMBER_OF_SENSOR_FINGER], 
                    received_data[NUMBER_OF_SENSOR_THUMB+NUMBER_OF_SENSOR_FINGER+1],
                    0.7 *received_data[NUMBER_OF_SENSOR_THUMB+NUMBER_OF_SENSOR_FINGER+2],
                    0.66 *0.7 *received_data[NUMBER_OF_SENSOR_THUMB+NUMBER_OF_SENSOR_FINGER+2]};

    return {thumbToSend[0], thumbToSend[1], thumbToSend[2], thumbToSend[3],
    indexToSend[0], indexToSend[1], indexToSend[2], indexToSend[3],
    middleToSend[0], middleToSend[1], middleToSend[2], middleToSend[3]};
    // Index the angles are {add/abd MCP PIP DIP}
    // Middle the angles are {add/abd MCP PIP DIP}
 }