#include <iostream>
#include <thread>
#include <chrono>
#include <array>          // For using std::array
#include <system_error>   // For handling system errors
#include "UDP_receiver.h"
#include <GL/glut.h>
#include "hand_display.h"

GLfloat cameraAngleX = 0.0f; // 摄像机绕X轴旋转的角度
GLfloat cameraAngleY = 0.0f; // 摄像机绕Y轴旋转的角度
GLfloat cameraDistance = 3.0f; // 摄像机距离观察目标点的距离

GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat rotate_x = 0.0f;
GLfloat rotate_y = 0.0f;

int main(){
    std::array<double, 16> data_to_send;
    data_to_send = {1.23, 3.45, 4.326, 5.672,
                    1.672, 3.45, 4.23, 5.326,
                    1.672, 3.326, 4.45, 5.23,
                    1.672, 3.326, 4.45, 5.23,
    };
    // UDP communication
    unsigned int port = 8080;
    udp::endpoint sender_endpoint;
    UdpCommunicator slaveDevice(port);
    while (true)
    {
        /** UPD sendering process*/
        try {
        std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER> received_data = slaveDevice.receive(sender_endpoint);

        // Print out the received values.
        std::copy(received_data.begin(), received_data.end(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << std::endl;
        
        slaveDevice.send(data_to_send, sender_endpoint);
        }
        catch(const std::system_error& e) {
            // Handle the exception
            std::cerr << "Caught system error: " << e.what() << '\n';
            // The program can continue running here
        }
        catch(const std::exception& e) {
            // Handle other standard exceptions
            std::cerr << "Caught exception: " << e.what() << '\n';
        }
        catch(...) {
            // Catch-all handler: can catch any exception
            std::cerr << "Caught unknown exception\n";
        }
        //std::string message(reinterpret_cast<char*>(received_data.data()), received_data.size() * sizeof(double));

        // std::cout << "Received message from Cadence: " << dataFromCadence[0]\
        //           <<", " << dataFromCadence[1] << std::endl;


        if(check_for_user_input()){
            break;
        }

    }
    slaveDevice.close();
    return 0;
}