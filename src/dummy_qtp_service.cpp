#include <rclcpp/rclcpp.hpp>
#include <motoros2_interfaces/srv/queue_traj_point.hpp>
#include <motoros2_interfaces/msg/queue_result_enum.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using QueueTrajPoint = motoros2_interfaces::srv::QueueTrajPoint;
using QueueResultEnum = motoros2_interfaces::msg::QueueResultEnum;

class DummyQTP : public rclcpp::Node
{
public:
  // constructor
  explicit DummyQTP(const std::string &node_name) : Node(node_name)
  {

    // Create the service to handle queue_traj_point requests
    this->queue_service_ = this->create_service<QueueTrajPoint>(
        "queue_traj_point", std::bind(&DummyQTP::handleQueueTrajPoint, this, std::placeholders::_1, std::placeholders::_2));

    this->publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/dummy_joint_states", 10);

    this->declare_parameter("response_delay", 0.05); // sleep time before accepting the point
    this->declare_parameter("busy_count", 10);       // sleep time before accepting the point
    this->get_parameter("response_delay", param_response_delay_);
    this->get_parameter("busy_count", param_busy_count_);
    response_delay_ = param_response_delay_.as_double();
    busy_count_ = param_busy_count_.as_int();
  }

private:
  rclcpp::Service<QueueTrajPoint>::SharedPtr queue_service_;
  int counter_ = 0, busy_count_ = 10;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Parameter param_response_delay_, param_busy_count_;
  double response_delay_;

  void handleQueueTrajPoint(const std::shared_ptr<QueueTrajPoint::Request> request,
                            std::shared_ptr<QueueTrajPoint::Response> response)
  {
    counter_++;
    // Dummy implementation of the service
    RCLCPP_INFO(get_logger(), "Received queue_traj_point request");

    // Check the request and set the response accordingly
    if (request->joint_names.size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "missing joint names");
      response->result_code.value = QueueResultEnum::INVALID_JOINT_LIST;
      response->message = QueueResultEnum::INVALID_JOINT_LIST_STR;
    }
    else if (request->point.positions.size() == 0 || request->point.velocities.size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "missing joint positions or velocities");
      response->result_code.value = QueueResultEnum::INVALID_JOINT_LIST;
      response->message = QueueResultEnum::INVALID_JOINT_LIST_STR;
    }
    else if (counter_ < busy_count_)
    {
      RCLCPP_INFO(get_logger(), "I am busy, try again later");
      response->result_code.value = QueueResultEnum::BUSY;
      response->message = QueueResultEnum::BUSY_STR;
    }
    else
    {
      counter_ = 0;
      auto message = sensor_msgs::msg::JointState();
      message.name = request->joint_names;
      message.position = request->point.positions;
      message.effort.resize(message.position.size(), 0.0);
      message.velocity.resize(message.position.size(), 0.0);
      publisher_->publish(message);
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long int>(response_delay_ * 1000)));
      RCLCPP_INFO(get_logger(), "Successfully queued the point");
      response->result_code.value = QueueResultEnum::SUCCESS;
      response->message = QueueResultEnum::SUCCESS_STR;
    }
    return;
  }
};

/*
  -------------------------------------------------------- main --------------------------------------------------------------------
*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DummyQTP>("dummy_qtp_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
