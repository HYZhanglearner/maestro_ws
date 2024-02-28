#include "finger.h"

// Constructor to initialize the finger's DOFs, segments, and starting position.
Finger::Finger(std::array<double, 4> DOFs, std::array<double, 3> segments,
               std::array<double, 3> startPos)
    : dof1_(DOFs[0]), dof2_(DOFs[1]), dof3_(DOFs[2]), dof4_(DOFs[3]),
      segment1_(segments[0]), segment2_(segments[1]), segment3_(segments[2]),
      startPosX_(startPos[0]), startPosY_(startPos[1]), startPosZ_(startPos[2]) {
    // Initialize other member variables or perform additional setup as needed.
}

// Function to determine if the finger is curved.
bool Finger::curved(std::string fingerName) {
    
    // if both joints angles are larger than threshold, the finger is curved
    if (fingerName == "thumb"){
        if(dof3_>1.0* 0.2 && dof4_> 0.9){
            return true; 
        }
        else{
            return false;
        }

    }
    else if(fingerName == "index"){
        if(dof1_>0.95* 0.16 && dof3_> 0.6* 2.0){
            return true; 
        }
        else{
            return false;
        }

    }
    else if(fingerName == "middle"){
        if(dof1_>0.95* 0.16 && dof3_> 0.6* 2.0){
            return true; 
        }
        else{
            return false;
        }

    }
    else{
        // do nothing
    }
    return false; // Placeholder return value.
}

// Function to create a transformation matrix using DH parameters

Eigen::Matrix4d Finger::createDHMatrix(double theta, double d, double a, double alpha) {
    double radTheta = theta;
    double radAlpha = alpha;

    Eigen::Matrix4d mat;
    mat << cos(radTheta), -sin(radTheta) * cos(radAlpha), sin(radTheta) * sin(radAlpha), a * cos(radTheta),
           sin(radTheta), cos(radTheta) * cos(radAlpha), -cos(radTheta) * sin(radAlpha), a * sin(radTheta),
           0, sin(radAlpha), cos(radAlpha), d,
           0, 0, 0, 1;
    return mat;
}

// Function to calculate the position of the fingertip using DH parameters
Eigen::Vector3d Finger::forwardKinematics_finger() {

    Eigen::Matrix4d Ori2Init {
                                {1, 0, 0, startPosX_},
                                {0, 1, 0, startPosY_},
                                {0, 0, 1, startPosZ_},
                                {0, 0, 0, 1}
                            }; 

    // Adjust MCP joint to include both rotations
    Eigen::Matrix4d T_MCP_Y = createYRotationMatrix(dof2_); // Rotation around Y-axis
    
    Eigen::Matrix4d T_MCP_X = createDHMatrix(0, 0, 0, dof1_); // Rotation around X-axis
    Eigen::Matrix4d T_MCP2PIP = createDHMatrix(0, segment1_, 0, dof3_); // Rotation around X-axis

    // PIP Joint
    Eigen::Matrix4d T_PIP2DIP = createDHMatrix(0, segment2_, 0, dof4_); // Rotation around X-axis

    // DIP Joint - angle dependent on PIP
    Eigen::Matrix4d T_DIP2TIP = createDHMatrix(0, segment3_, 0, 0);

    // Compute the final transformation matrix
    Eigen::Matrix4d T_Final = T_MCP_Y * T_MCP_X * T_MCP2PIP *
                                T_PIP2DIP* T_DIP2TIP;

    // Multiply with the initial position (assuming the base position is at the origin)
    Eigen::Matrix4d finalPosition4D = Ori2Init * T_Final;

    // Convert to 3D vector
    Eigen::Vector3d finalPosition3D = {finalPosition4D(0, 3),
                                        finalPosition4D(1, 3),
                                        finalPosition4D(2, 3)};
    return finalPosition3D;
}

void Finger::updateJoint(std::vector<double> newJoints){
    dof1_ = newJoints[0];
    dof2_ = newJoints[1];
    dof3_ = newJoints[2];
    dof4_ = newJoints[3];

}

Eigen::Matrix4d Finger::createYRotationMatrix(double angle) {
    double rad = angle;
    Eigen::Matrix4d mat;
    mat << cos(rad), 0, sin(rad), 0,
           0, 1, 0, 0,
           -sin(rad), 0, cos(rad), 0,
           0, 0, 0, 1;
    return mat;
}

Eigen::Vector3d Finger::forwardKinematics_thumb() {
    
    // Adjust MCP joint to include both rotations
    Eigen::Matrix4d Ori2Init {
                                {1, 0, 0, startPosX_},
                                {0, 1, 0, startPosY_},
                                {0, 0, 1, startPosZ_},
                                {0, 0, 0, 1}
                            }; 
    Eigen::Matrix4d T_MCP_X = createDHMatrix(0, 0, 0, dof2_); // Rotation around X-axis
    
    Eigen::Matrix4d T_MCP_Y = createYRotationMatrix(dof1_); // Rotation around Y-axis
    Eigen::Matrix4d T_MCP_Z = createDHMatrix(0, segment1_, 0, 0); // Translation in Z-axis

    // PIP Joint
    Eigen::Matrix4d T_PIP_Y = createYRotationMatrix(dof3_); // Rotation around Y-axis
    Eigen::Matrix4d T_PIP_Z = createDHMatrix(0, segment2_, 0, 0);
    
    // DIP Joint - angle dependent on PIP
    Eigen::Matrix4d T_DIP_Y = createYRotationMatrix(dof4_); // Rotation around Y-axis
    Eigen::Matrix4d T_DIP = createDHMatrix(0, segment3_, 0, 0);

    // Compute the final transformation matrix
    Eigen::Matrix4d T_Final =  T_MCP_X* T_MCP_Y* T_MCP_Z* T_PIP_Y* T_PIP_Z* T_DIP_Y* T_DIP;
    // Eigen::Matrix4d T_Final =  T_MCP_X* T_MCP_Y* T_MCP_Z;
    // Multiply with the initial position (assuming the base position is at the origin)
    Eigen::Matrix4d finalPosition4D = Ori2Init *T_Final;

    // Convert to 3D vector
    Eigen::Vector3d finalPosition3D = {finalPosition4D(0, 3),
                                        finalPosition4D(1, 3),
                                        finalPosition4D(2, 3)};
    return finalPosition3D;
}