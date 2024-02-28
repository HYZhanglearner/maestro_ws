#ifndef FINGER_H
#define FINGER_H

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <cmath>

class Finger {
public:
    // Constructor to initialize the finger's degrees of freedom (DOFs),
    // segments, and starting position.
    Finger(std::array<double, 4> DOFs, std::array<double, 3> segments,
               std::array<double, 3> startPos);

    // Function to determine if the finger is curved.
    bool curved(std::string fingerName); // 

    // Function to find the position of the tip of the finger.
    Eigen::Vector3d forwardKinematics_finger(); //
    Eigen::Vector3d forwardKinematics_thumb(); //
    // Function to create a transformation matrix using DH parameters
    Eigen::Matrix4d createDHMatrix(double theta, double d, double a, double alpha);

    Eigen::Matrix4d createYRotationMatrix(double angle);
    
    void updateJoint(std::vector<double> newJoints);

    // Degrees of freedom (DOFs) for the finger.
    double dof1_;
    double dof2_;
    double dof3_;
    double dof4_;

private:
    // Length of segments for the finger.
    double segment1_;
    double segment2_;
    double segment3_;

    // Starting position of the finger.
    double startPosX_;
    double startPosY_;
    double startPosZ_;
};

#endif // FINGER_H
