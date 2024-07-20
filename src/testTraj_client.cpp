#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaskawa_executor/action/traj_executor.hpp>
#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using TrajExecutor = yaskawa_executor::action::TrajExecutor; // our action (we're the client here)

class TrajectoryActionClient : public rclcpp::Node
{
public:
  using ClientGoalHandleTE = rclcpp_action::ClientGoalHandle<TrajExecutor>;

  // constructor
  explicit TrajectoryActionClient() : Node("trajectory_action_client")
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("yaskawa_executor_tests");

    this->declare_parameter("local_csv_path", "/config/solutions_60p.csv");                                                                                                             // where to get the joint trajectory
    this->declare_parameter("joint_names", std::vector<std::string>{"group_1/joint_1", "group_1/joint_2", "group_1/joint_3", "group_1/joint_4", "group_1/joint_5", "group_1/joint_6"}); // joint names for the action

    this->get_parameter("local_csv_path", param_csv_path_);
    this->get_parameter("joint_names", param_joint_names_);

    csv_path_ = package_share_directory + param_csv_path_.as_string();
    desired_joint_names_ = param_joint_names_.as_string_array();
    std::cout << csv_path_ << std::endl;

    // create the client
    this->action_client_te_ = rclcpp_action::create_client<TrajExecutor>(this, "/trajectory_action_server");
    // wait for the server
    if (!action_client_te_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "TE not available after waiting");
      return;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Connection established with TE");
    }
    // Create and prepare the goal
    auto action_goal = TrajExecutor::Goal();
    if (!readCSV(csv_path_))
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot read the csv");
      return;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "csv file is correctly formatted");
    }
    if (!fillActionGoal(action_goal))
    {
      RCLCPP_ERROR(this->get_logger(), "Can't create TE goal");
      return;
    }
    // send it to TE
    auto send_goal_options = rclcpp_action::Client<TrajExecutor>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TrajectoryActionClient::te_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&TrajectoryActionClient::te_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&TrajectoryActionClient::te_result_callback, this, std::placeholders::_1);

    // send the goal
    auto future_goal_handle = action_client_te_->async_send_goal(action_goal, send_goal_options);
  }

private:
  rclcpp_action::Client<TrajExecutor>::SharedPtr action_client_te_;
  std::vector<std::string> desired_joint_names_;
  std::vector<std::string> csvContents_;
  rclcpp::Parameter param_csv_path_, param_joint_names_;
  std::string csv_path_ = "";

  bool fillActionGoal(TrajExecutor::Goal &goal_follow)
  {
    // Set up the trajectory header
    goal_follow.joint_trajectory.header.stamp = rclcpp::Time(0); // Use current time
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> all_points;

    // work on the csv
    goal_follow.joint_trajectory.joint_names = desired_joint_names_;
    for (unsigned i = 0; i < csvContents_.size(); i++)
    {
      all_points.push_back(transformToPoint(csvContents_[i]));
      all_points.back().velocities.resize(all_points.back().positions.size(), 0.0);
    }
    goal_follow.joint_trajectory.points = all_points;
    return true; // Successfully filled the goal
  }

  // callback for when the yaskawa_executor accepts the goal
  void te_response_callback(ClientGoalHandleTE::SharedPtr future)
  {
    auto goal_handle = future.get();
    RCLCPP_INFO(this->get_logger(), "response to sent goal=%d", static_cast<int>(goal_handle->get_status()));
  }

  // callback for feedback from yaskawa_executor
  void te_feedback_callback(ClientGoalHandleTE::SharedPtr, const std::shared_ptr<const TrajExecutor::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "got feedback: moving=%s; curr_point=%d/%d", feedback->moving ? "true" : "false", feedback->current_point, feedback->total_points);
  }

  // callback for final result from yaskawa_executor
  void te_result_callback(const ClientGoalHandleTE::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "result is SUCCESS");
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_INFO(this->get_logger(), "result is CANCELLED");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "result is ABORT");
    }
    RCLCPP_INFO(this->get_logger(), "received message = %s", result.result->result_comment.c_str());

    // Shutdown the node as soon as the result is received
    rclcpp::shutdown();
  }

  // read the csv file and validate contents
  bool readCSV(const std::string &file_path)
  {
    // Read the CSV file and fill the trajectory points
    std::ifstream csv_file(file_path);
    if (!csv_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "the csv file is empty.");
      return false; // Abort if the CSV file cannot be opened
    }

    std::string line;
    while (std::getline(csv_file, line))
    {
      if (!validateLine(line))
      {
        RCLCPP_ERROR(this->get_logger(), "wrong amount of columns in the csv file.");
        return false;
      }
      csvContents_.push_back(line); // we simply record all lines here
      RCLCPP_INFO(this->get_logger(), "lines in csv=%zu", csvContents_.size());
    }
    return true;
  }

  // check amount of columns in each line of csv
  bool validateLine(const std::string &line)
  {
    std::istringstream stream(line);
    std::string token;
    unsigned int columnCount = 0;
    while (std::getline(stream, token, ','))
    {
      ++columnCount;
    }
    return columnCount == desired_joint_names_.size() + 1;
  }

  // parse line
  trajectory_msgs::msg::JointTrajectoryPoint transformToPoint(const std::string &line)
  {
    trajectory_msgs::msg::JointTrajectoryPoint temp_point;
    std::istringstream iss(line);
    std::string token;
    std::cout << line << std::endl;
    for (size_t i = 0; i < desired_joint_names_.size(); ++i)
    {
      if (!(iss >> token))
      {
        // Handle error, e.g., log and return an invalid point
        RCLCPP_ERROR(get_logger(), "Error reading joint position from input line");
        return temp_point; // Returning an invalid point
      }

      // std::cout<<std::stod(token)<<std::endl;
      double joint_position = std::stod(token);
      temp_point.positions.push_back(joint_position);
    }

    // Calculate time_from_start based on the last column and time_to_approach
    if (!(iss >> token))
    {
      // Handle error, e.g., log and return an invalid point
      RCLCPP_ERROR(get_logger(), "Error reading time_from_start from input line");
      return temp_point; // Returning an invalid point
    }

    temp_point.time_from_start = rclcpp::Duration::from_seconds(std::stod(token));

    return temp_point;
  }
};

/*
  -------------------------------------------------------- main --------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}