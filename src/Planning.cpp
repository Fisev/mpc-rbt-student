#include "../include/Planning.hpp"

using namespace std::chrono_literals;

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Service for path
        // add code here
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("plan_path", std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher for path
        // add code here
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("map_server/map");

        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        
        while (!map_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Planning node started.");
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here
    auto response = future.get();
    if(response){
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received");
        RCLCPP_INFO(get_logger(), "Map size: width = %d, height = %d", map_.info.width, map_.info.height);
    } else{
        RCLCPP_INFO(get_logger(), "Failed to receive map");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

    RCLCPP_INFO(get_logger(), "Received path planning request from (%f, %f) to (%f, %f)",
    request->start.pose.position.x, request->start.pose.position.y,
    request->goal.pose.position.x, request->goal.pose.position.y);

    aStar(request->start, request->goal);
    //smoothPath();
    path_pub_->publish(path_);
    response->plan = path_;
}

void PlanningNode::dilateMap() {
    // add code here
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    auto resolution = map_.info.resolution;
    auto origin = map_.info.origin;
    
    int width = map_.info.width;
    int height = map_.info.height;
    
    int sx = static_cast<int>((start.pose.position.x - origin.position.x) / resolution);
    int sy = static_cast<int>((start.pose.position.y - origin.position.y) / resolution);
    int gx = static_cast<int>((goal.pose.position.x - origin.position.x) / resolution);
    int gy = static_cast<int>((goal.pose.position.y - origin.position.y) / resolution);
    
    Cell cStart(sx, sy);
    Cell cGoal(gx, gy);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));
    
    auto getIndex = [width](int x, int y) {
    	return y*width+x;
    };
    
    auto isObstacle = [this](int x, int y) {
    	return map_.data[y * map_.info.width + x] > 50;
    };
    
    std::vector<std::pair<int, int>> directions = {
    	{1, 0}, {-1, 0}, {0, 1}, {0, -1},
    	{1, 1}, {-1,-1}, {1,-1}, {-1, 1}
    };

    while(!openList.empty() && rclcpp::ok()) {
        auto current_it = std::min_element(openList.begin(), openList.end(), [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b){
        	return a->f < b->f;
        	});
        std::shared_ptr<Cell> current = *current_it;
        openList.erase(current_it);
        
        if(current->x == cGoal.x && current->y == cGoal.y) {
        	path_.poses.clear();
        	while(current) {
        		geometry_msgs::msg::PoseStamped pose;
        		pose.pose.position.x = current->x*resolution+origin.position.x+resolution/2;
        		pose.pose.position.y = current->y*resolution+origin.position.y+resolution/2;
        		pose.pose.position.z = 0.0;
        		pose.pose.orientation.w = 1.0;
        		path_.poses.push_back(pose);
        		current = current->parent;
        	}
        	
        	std::reverse(path_.poses.begin(), path_.poses.end());
        	path_.header.frame_id = map_.header.frame_id;
        	return;
        }
        
        closedList[getIndex(current->x, current->y)] = true;
        
        for(const auto &[dx, dy] : directions) {
        	int nx = current->x + dx;
        	int ny = current->y + dy;
        	
        	if(nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if(closedList[getIndex(nx, ny)]) continue;
        	if(isObstacle(nx, ny)) continue;
            

        		
        	float g_new = current->g + std::hypot(dx, dy);
        	float h_new = std::hypot(gx - nx, gy - ny);
        	float f_new = g_new + h_new;
        	
        	auto it = std::find_if(openList.begin(), openList.end(), [nx, ny](const std::shared_ptr<Cell> &cell) {
        		return cell->x == nx && cell->y == ny;
        	});
        	
        	if (it == openList.end() || f_new < (*it)->f) {
        		auto neighbor = std::make_shared<Cell>(nx, ny);
        		neighbor->g = g_new;
        		neighbor->h = h_new;
        		neighbor->f = f_new;
        		neighbor->parent = current;
        		
        		if(it == openList.end()) {
        			openList.push_back(neighbor);
        		} else {
        			*it = neighbor;
        		}
        	}	
        }
        
         
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
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
   x = c;
   y = r;
}









