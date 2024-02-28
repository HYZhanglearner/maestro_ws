#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "finger.h"
std::array<double, 12> receivedData;
bool shouldExit = false;

void messageCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() == 12)
    {
        for (size_t i = 0; i < 12; ++i)
        {
            receivedData[i] = msg->data[i];
        }
        // You can now use the receivedData array as needed
        RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Received data");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("subscriber_node"), "Received message has incorrect size");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hand_pattern_recog_node"); // Use the same node name as in CMakeLists.txt

    auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("hand_joint_space", 10, messageCallback); // Specify the message type and topic name

    auto publisher = node->create_publisher<std_msgs::msg::String>("hand_pose_cmd", 10);

    Finger thumb({0.0f, 0.0f, 0.0f, 0.0f},
                 {0.45f, 0.35f, 0.28f},
                 {0.0f, 0.0f, -0.7f});

    Finger index({0.0f, 0.0f, 0.0f, 0.0f},
                 {0.45f, 0.27f, 0.18f},
                 {0.0, 0.0f, 0});

    Finger middle({0.0f, 0.0f, 0.0f, 0.0f},
                  {0.45f, 0.27f, 0.18f},
                  {0.3f, 0.0f, 0.0});

    while (rclcpp::ok() && !shouldExit)
    {
        rclcpp::spin_some(node); // Process any available messages
        Eigen::Vector3d end_posi_thumb;
        Eigen::Vector3d end_posi_index;
        Eigen::Vector3d end_posi_middle;

        // Update joint space and get end effector position
        // Thumb update:
        thumb.updateJoint({receivedData[0], receivedData[1], receivedData[2], receivedData[3]});
        end_posi_thumb = thumb.forwardKinematics_thumb();
        // RCLCPP_INFO(node->get_logger(), "Thumb: end x: %f, y: %f, z: %f",
        //             end_posi_thumb.x(), end_posi_thumb.y(), end_posi_thumb.z());

        // Index update:
        index.updateJoint({receivedData[4], receivedData[5], receivedData[6], receivedData[7]});
        end_posi_index = index.forwardKinematics_finger();
        // RCLCPP_INFO(node->get_logger(), "Index: end x: %f, y: %f, z: %f",
        //             end_posi_index.x(), end_posi_index.y(), end_posi_index.z());

        // Middle update:
        middle.updateJoint({receivedData[8], receivedData[9], receivedData[10], receivedData[11]});
        end_posi_middle = middle.forwardKinematics_finger();
        // RCLCPP_INFO(node->get_logger(), "Middle: end x: %f, y: %f, z: %f",
        //             end_posi_middle.x(), end_posi_middle.y(), end_posi_middle.z());

        // Detecting curving of fingers
        bool is_thumb_curved = thumb.curved("thumb");
        bool is_index_curved = index.curved("index");
        bool is_middle_curved = middle.curved("middle");
        // RCLCPP_INFO(node->get_logger(), "%d, %d, %d",is_thumb_curved, is_index_curved, is_middle_curved);
        // Define the msg send to "hand_pose cmd"
        auto message = std_msgs::msg::String();

        // Standard format "hand/arm_position/joint7/orient/grasp_value"
        // Grasping
        if (!is_thumb_curved && !is_index_curved && is_middle_curved)
        {
            // Calculate the difference vector
            Eigen::Vector3d diff = end_posi_thumb - end_posi_index;
            double squaredDistance = diff.squaredNorm();
            double distance = std::sqrt(squaredDistance);
            // 0.4 is a norm, it could be set to different values.
            double value = distance / 1.6;
            std::string strValue = std::to_string(value);
            message.data = "hand_grasp_" + strValue;
            publisher->publish(message);
            std::string reminder = "Grasping cmd, value: " + strValue;
            RCLCPP_INFO(node->get_logger(), reminder.c_str());
        }
        // Arm motion
        else if (is_thumb_curved && !is_index_curved && !is_middle_curved)
        {
            // Joint 7 motion
            if (abs(end_posi_index.y() - end_posi_middle.y()) >= 0.5)
            {
                RCLCPP_INFO(node->get_logger(), "In arm setting, joint 7 control.");
                if ((end_posi_index.y() - end_posi_middle.y()) > 0.0)
                {
                    message.data = "arm_joint7_clockwise";
                    publisher->publish(message);
                    RCLCPP_INFO(node->get_logger(), "arm_joint7_clockwise");
                }
                else if ((end_posi_index.y() - end_posi_middle.y()) < 0.0)
                {
                    message.data = "arm_joint7_counterClockwise";
                    publisher->publish(message);
                    RCLCPP_INFO(node->get_logger(), "arm_joint7_counterClockwise");
                }
            }
            // ee position
            else if (abs(end_posi_index.y() - end_posi_middle.y()) <= 0.5)
            {
                RCLCPP_INFO(node->get_logger(), "In arm setting, ee postion control.");
                // index and middle fingers are flatten
                if ((abs(index.dof1_) <= 0.2) && (abs(index.dof3_) <= 0.35) &&
                    (abs(middle.dof1_) <= 0.2) && (abs(middle.dof3_) <= 0.35))
                {
                    if ((index.dof2_ + middle.dof2_) / 2 < -0.15)
                    {
                        message.data = "arm_position_leftward";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "arm_position_leftward");
                    }
                    else if ((index.dof2_ + middle.dof2_) / 2 > 0.0)
                    {
                        message.data = "arm_position_rightward";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "arm_position_rightward");
                    }
                    else
                    {
                        message.data = "";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "Flat finger, no left or right.");
                    }
                }
                // MPC move PIP flat
                else
                {
                    if ((index.dof1_ + middle.dof1_) / 2 > 0.2f)
                    {
                        message.data = "arm_position_downward";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "arm_position_downward.");
                    }
                    else if ((index.dof1_ + middle.dof1_) / 2 < -0.12f)
                    {
                        message.data = "arm_position_upward";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "arm_position_upward.");
                    }
                    else
                    {
                        message.data = "";
                        publisher->publish(message);
                        RCLCPP_INFO(node->get_logger(), "In up down control, no direction detected.");
                    }
                }
            }
            else
            {
                message.data = "";
                publisher->publish(message);
                RCLCPP_INFO(node->get_logger(), "In arm motion, but no cmd detected.");
            }
        }
        // front and back motion
        else if (!is_thumb_curved && is_index_curved && is_middle_curved)
        {
            RCLCPP_INFO(node->get_logger(), "In front or back mode.");
            // MPC move PIP bend
            if ((thumb.dof3_ > 0.05f) && (thumb.dof4_ > 0.6f))
            {
                message.data = "arm_position_forward";
                publisher->publish(message);
                RCLCPP_INFO(node->get_logger(), "arm_position_forward");
            }
            else if ((thumb.dof3_ < 0.05f) && (thumb.dof4_ < 0.05f))
            {
                message.data = "arm_position_backward";
                publisher->publish(message);
                RCLCPP_INFO(node->get_logger(), "arm_position_backward.");
            }
            else
            {
                message.data = "";
                publisher->publish(message);
                RCLCPP_INFO(node->get_logger(), "In front or back mode, no direction detected.");
            }
        }
        // Thumb and middle are curved
        else if (is_thumb_curved && !is_index_curved && is_middle_curved)
        {
            std::vector<double> numbers = {0.0f, index.dof1_, index.dof2_};
            // Create a stringstream to build the string
            std::stringstream ss;
            // Iterate through the vector and append each element to the stringstream
            // Start the string with an opening brace
            ss << "{";

            // Iterate through the vector and append each element to the stringstream
            for (size_t i = 0; i < numbers.size(); ++i)
            {
                ss << numbers[i];
                if (i < numbers.size() - 1)
                {
                    // Add a comma after all but the last element
                    ss << ",";
                }
            }

            // Close the string with a closing brace
            ss << "}";
            // Get the concatenated string from the stringstream
            std::string strValue = ss.str();
            message.data = "arm_orient_" + strValue;
            publisher->publish(message);
            RCLCPP_INFO(node->get_logger(), message.data.c_str());
        }
        else
        {
            message.data = "";
            publisher->publish(message);
            RCLCPP_INFO(node->get_logger(), "Stay on current posture");
        }

        publisher->publish(message);
    }
    rclcpp::shutdown();
    return 0;
}