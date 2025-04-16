#include "../include/Planning.hpp"

using namespace std::chrono_literals;

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Service for path
        // add code here
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("/plan_path", std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher for path
        // add code here
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        
        while (!map_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));

        // add code here

        RCLCPP_INFO(get_logger(), "Planning node started.");
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here
    auto response = future.get();
    if(response){
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received");
    } else{
        RCLCPP_INFO(get_logger(), "Failed to receive map");
    }
    // ********
    // * Help *
    // ********
    /*
   
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // add code here

    // ********
    // * Help *
    // ********
    
    aStar(request->start, request->goal);
    smoothPath();
    response->plan = path_;
    path_pub_->publish(path_);
    
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here

    // ********
    // * Help *
    // ********

    Cell cStart((int)(start.pose.position.x - map_.info.origin.position.x)/map_.info.resolution, (int)(start.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);      // meaby shoul subtract map pos
    Cell cGoal((int)(goal.pose.position.x - map_.info.origin.position.x)/map_.info.resolution, (int)(goal.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);

    . openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
    // Find the cell with the lowest f cost 
        auto current = std::min_element(openList.begin(), openList.end(), [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>&
        // If the goal is reached, reconstruct the path
        if ((*current)->x == cGoal.x && (*current)->y == cGoal.y) {
            // Reconstruct path
             std::vector<Cell> path;
             auto cell = *current;
             while (cell) {
                 path.push_back(*cell);
                 cell = cell->parent;
             }
             std::reverse(path.begin(), path.end());
             // Publish or use the path
             return;
        }
/*
        // Move the current cell from open to closed list
         closedList[(*current)->y * map_.info.width + (*current)->x] = true;
         openList.erase(current);

         // Generate neighbors
         std::vector<std::shared_ptr<Cell>> neighbors = generateNeighbors(*current);

         for (auto& neighbor : neighbors) {
             if (closedList[neighbor->y * map_.info.width + neighbor->x]) {
                continue;
             }

            // Calculate the costs
             neighbor->gCost = (*current)->gCost + distance(*current, *neighbor);
             neighbor->hCost = distance(*neighbor, cGoal);
             neighbor->fCost = neighbor->gCost + neighbor->hCost;
             neighbor->parent = *current;

            // Add to open list if not already present
             if (std::find_if(openList.begin(), openList.end(), &neighbor {
                 return cell->x == neighbor->x && cell->y == neighbor->y;
                }) == openList.end()) {
                 openList.push_back(neighbor);
             }
         }
     }
*/
    //RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
}
