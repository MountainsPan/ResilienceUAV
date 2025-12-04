#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <cmath>

#include <chrono>
#include <iostream>
 
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
 
class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
 
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
 
        offboard_setpoint_counter_ = 0;
 
        auto timer_callback = [this]() -> void {
 
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
 
                // Arm the vehicle
                this->arm();
            }
 
             // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
 
             // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }
 
    void arm();
    void disarm();
 
 private:
     rclcpp::TimerBase::SharedPtr timer_;
 
     rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
     rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
     rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
 
     std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
 
     uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
 
     void publish_offboard_control_mode();
     void publish_trajectory_setpoint();
     void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
 };
 
 /**
  * @brief Send a command to Arm the vehicle
  */
 void OffboardControl::arm()
 {
     publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
 
     RCLCPP_INFO(this->get_logger(), "Arm command send");
 }
 
 /**
  * @brief Send a command to Disarm the vehicle
  */
 void OffboardControl::disarm()
 {
     publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
 
     RCLCPP_INFO(this->get_logger(), "Disarm command send");
 }
 
 /**
  * @brief Publish the offboard control mode.
  *        For this example, only position and altitude controls are active.
  */
 void OffboardControl::publish_offboard_control_mode()
 {
     OffboardControlMode msg{};
     msg.position = true;
     msg.velocity = false;
     msg.acceleration = false;
     msg.attitude = false;
     msg.body_rate = false;
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     offboard_control_mode_publisher_->publish(msg);
 }
 
 /**
  * @brief Publish a trajectory setpoint
  *        For this example, it sends a trajectory setpoint to make the
  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
  */
 void OffboardControl::publish_trajectory_setpoint()
 {
     TrajectorySetpoint msg{};
     static double step = 0.0; // 轨迹步长
     msg.position = {0.0, static_cast<float>(2*step), -5.0};
     msg.yaw = -3.14; // [-PI:PI]
     step += 1;
    
     static double angle = 0.0; // 当前角度（弧度）
     static int phase = 0;      // 阶段：0 表示盘旋，1 表示下降

    if (phase == 0) {
        // 盘旋阶段
        float radius = 25.0f; // 圆的半径（单位：米）
        msg.position = {static_cast<float>(radius * cos(angle)), 
                        static_cast<float>(radius * sin(angle)), 
                        -5.0f}; // 圆形轨迹，保持高度 5 米
        msg.yaw = static_cast<float>(angle); // 让无人机的朝向随轨迹变化
        angle += 0.1; // 每次调用增加角度（调整步长控制速度）
        }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    
    trajectory_setpoint_publisher_->publish(msg);
 }
 
 /**
  * @brief Publish vehicle commands
  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
  * @param param1    Command parameter 1
  * @param param2    Command parameter 2
  */
 void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
 {
     VehicleCommand msg{};
     msg.param1 = param1;
     msg.param2 = param2;
     msg.command = command;
     msg.target_system = 1;
     msg.target_component = 1;
     msg.source_system = 1;
     msg.source_component = 1;
     msg.from_external = true;
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     vehicle_command_publisher_->publish(msg);
 }
 
 int main(int argc, char *argv[])
 {
     std::cout << "Starting offboard control node..." << std::endl;
     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<OffboardControl>());
 
     rclcpp::shutdown();
     return 0;
 }
 