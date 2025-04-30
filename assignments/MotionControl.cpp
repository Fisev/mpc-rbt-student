#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node")
{
    // Subscriber for odometry
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

    // Subscriber for laser scan
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/tiago_base/Hokuyo_URG_04LX_UG01", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

    // Publisher for cmd_vel
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Client for path planning (uses /plan_path service)
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    // Action server for /go_to_goal
    nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
        std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motion control node started.");

    // Connect to planning service
    while (!plan_client_->wait_for_service(std::chrono::seconds(2)) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for path planner...");
    }
}


void MotionControlNode::checkCollision() {
    const double threshold = 0.3;
    const auto& ranges = laser_scan_.ranges;

    if (ranges.empty()) {
        RCLCPP_WARN(get_logger(), "Laser scan data is empty. Cannot check for collisions.");
        return;
    }

    int center = static_cast<int>(ranges.size()) / 2;
    int window = 10;

    for (int i = center - window; i <= center + window && i < static_cast<int>(ranges.size()); ++i) {
        if (i < 0) continue;

        double range = ranges[i];
        if (range > 0.0 && range < threshold) {
            if (!collision_detected_) {
                RCLCPP_WARN(get_logger(), "Obstacle detected! Emergency stop triggered at scan index %d", i);
                collision_detected_ = true;

                twist_publisher_->publish(geometry_msgs::msg::Twist());

                if (goal_handle_ && goal_handle_->is_active()) {
                    goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
                    RCLCPP_INFO(get_logger(), "Goal aborted due to collision.");
                }
            }
            return;
        }
    }

    // No obstacle detected in central region
    collision_detected_ = false;
}





void MotionControlNode::updateTwist(const geometry_msgs::msg::Pose &target_pose) {
    double dx = target_pose.position.x - current_pose_.pose.position.x;
    double dy = target_pose.position.y - current_pose_.pose.position.y;

    double desired_heading = std::atan2(dy, dx);

    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double heading_error = desired_heading - yaw;
    while (heading_error > M_PI) heading_error -= 2*M_PI;
    while (heading_error < -M_PI) heading_error += 2*M_PI;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.15;  // max speed dopředu
    twist.angular.z = 1.5 * heading_error; // P řízení směru

    twist_publisher_->publish(twist);
}


 /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
   rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) 
{
    (void)uuid;  // Pokud UUID nepoužíváme

    if (!rclcpp::ok()) {
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Validace cíle, např.:
    if (goal->pose.pose.position.x != goal->pose.pose.position.x ||  // NaN check
        goal->pose.pose.position.y != goal->pose.pose.position.y) {
        RCLCPP_WARN(this->get_logger(), "Invalid goal received (NaN).");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Logg
    RCLCPP_INFO(this->get_logger(), "Received goal: [x: %.2f, y: %.2f]",
                goal->pose.pose.position.x, goal->pose.pose.position.y);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


    /*
    (void)uuid;
    ...
    return ...;
    */
rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
    /*
    (void)goal_handle;
    ...
    return ...;
    */
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) 
{
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;

    // Vytvoření požadavku na plánovač trasy
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;  // musí být dříve aktualizován pomocí odometrie!
    request->goal = goal_pose_;
    request->tolerance = 0.0;

    // Odeslání požadavku a přiřazení callbacku
    auto future = plan_client_->async_send_request(
        request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
    );
}
    /*
    ...
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
    */



void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(100.0); // 10 Hz

    size_t target_idx = 0;

    while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {

        if (collision_detected_) {
            RCLCPP_WARN(get_logger(), "Collision detected during execution. Stopping.");
            return;
        }

        if (target_idx >= path_.poses.size()) {
            break; // Všechno dosaženo
        }

        const auto& target_pose = path_.poses[target_idx].pose;

        double dx = target_pose.position.x - current_pose_.pose.position.x;
        double dy = target_pose.position.y - current_pose_.pose.position.y;
        double distance = std::hypot(dx, dy);

        if (distance < 0.1) {
            ++target_idx;
            continue;
        }

        // Ovládání - nově
        updateTwist(target_pose);  // Už nic nevrací, sama publikuje!

        // Feedback
        auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        feedback->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback);

        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_INFO(get_logger(), "Goal canceled.");
            return;
        }

        loop_rate.sleep();
    }

    // Stop robot
    twist_publisher_->publish(geometry_msgs::msg::Twist());
    if (goal_handle_ && goal_handle_->is_active()) {
        goal_handle_->succeed(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
        RCLCPP_INFO(get_logger(), "Goal reached.");
    }
}
    
    /*
    rclcpp::Rate loop_rate(1.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {
            ...
            return;
        }

        ...

        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
    */



    void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
        auto response = future.get();
        if (response && response->plan.poses.size() > 0) {
            path_ = response->plan;
    
            // NEZAČÍNAT OKAMŽITĚ
            if (collision_detected_) {
                RCLCPP_ERROR(get_logger(), "Collision detected before execution. Goal aborted.");
                goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
                return;
            }
    
            if (current_pose_.header.stamp.sec == 0) { // Pokud timestamp je 0 → nikdy nepřišla odometrie
                RCLCPP_ERROR(get_logger(), "Current pose not available! Goal aborted.");
                goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
                return;
            }
    
            RCLCPP_INFO(get_logger(), "Starting motion execution.");
            std::thread(&MotionControlNode::execute, this).detach();
    
        } else {
            RCLCPP_ERROR(get_logger(), "Path planning failed or returned empty path.");
            goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
        }
    }
    
    /*
    if (response && response->plan.poses.size() > 0) {
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    }
    */


    void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
        current_pose_.pose = msg.pose.pose;
        current_pose_.header = msg.header;
    
        checkCollision();
    }
    


void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
