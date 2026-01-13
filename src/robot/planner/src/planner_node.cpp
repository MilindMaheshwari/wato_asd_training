#include "planner_node.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  // Initialize state
  state_ = State::WAITING_FOR_GOAL;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(this->get_logger(), "Goal received! Goal: x: %f, y: %f", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}


struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  double g_score;  // Distance from start
  double h_score;  // Distance from goal
  AStarNode *parent; // Pointer to parent node

  AStarNode(CellIndex idx, double g, double h, AStarNode *parent_node) : 
    index(idx), g_score(g), h_score(h), parent(parent_node), f_score(g + h) {}
  

};
 

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

nav_msgs::msg::Path PlannerNode::planPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return path;
  }

  // Create obstacle map (0 for free, 1 for occupied)
  // Using std::vector instead of VLA for standard C++ compliance
  std::vector<int> obstacle_map(current_map_.data.size());
  for (size_t i = 0; i < current_map_.data.size(); ++i) {
    obstacle_map[i] = (current_map_.data[i] > 0) ? 1 : 0;
  }

  // Heuristic: Euclidean distance
  auto heuristic = [](const CellIndex& a, const CellIndex& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  // Convert world coordinates to grid coordinates
  // Uses map origin to correctly map world to cell
  CellIndex start(
    (int)((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution),
    (int)((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution)
  );

  CellIndex goal(
    (int)((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution),
    (int)((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution)
  );

  auto isValid = [&](const CellIndex& idx) {
    return idx.x >= 0 && idx.x < (int)current_map_.info.width &&
           idx.y >= 0 && idx.y < (int)current_map_.info.height;
  };

  if (!isValid(start) || !isValid(goal)) {
    RCLCPP_WARN(this->get_logger(), "Start or Goal is out of map bounds!");
    return path;
  }

  // Priority Queue for Open Set
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  
  // Hash Maps for G-Score and Came-From
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  // Initialize start
  g_score[start] = 0.0;
  open_set.emplace(start, 0.0, heuristic(start, goal), nullptr);

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      // Reconstruct path
      std::vector<geometry_msgs::msg::PoseStamped> poses;
      CellIndex curr = goal;
      
      while (curr != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = curr.x * current_map_.info.resolution + current_map_.info.origin.position.x;
        pose.pose.position.y = curr.y * current_map_.info.resolution + current_map_.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);
        
        curr = came_from[curr];
      }
      
      // Add start
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.header = path.header;
      start_pose.pose.position.x = start.x * current_map_.info.resolution + current_map_.info.origin.position.x;
      start_pose.pose.position.y = start.y * current_map_.info.resolution + current_map_.info.origin.position.y;
      start_pose.pose.orientation.w = 1.0;
      poses.push_back(start_pose);

      std::reverse(poses.begin(), poses.end());
      path.poses = poses;
      path_pub_->publish(path);
      return path;
    }

    if (g_score.count(current.index) && current.g_score > g_score[current.index]) {
      continue;
    }

    // Lambda to get valid neighbors (8-connected)
    auto getNeighbors = [&](const CellIndex& current) -> std::vector<CellIndex> {
      std::vector<CellIndex> neighbors;
      // 8-connected neighbors
      const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
      const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

      for (int i = 0; i < 8; ++i) {
        CellIndex neighbor(current.x + dx[i], current.y + dy[i]);

        if (isValid(neighbor)) {
            // Check occupancy using obstacle_map
            int map_idx = neighbor.y * current_map_.info.width + neighbor.x;
            if (obstacle_map[map_idx] == 0) {
               neighbors.push_back(neighbor);
            }
        }
      }
      return neighbors;
    };

    for (const CellIndex& neighbor : getNeighbors(current.index)) {
      double move_cost = std::hypot(neighbor.x - current.index.x, neighbor.y - current.index.y);
      double tentative_g = g_score[current.index] + move_cost;

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double h = heuristic(neighbor, goal);
        open_set.emplace(neighbor, tentative_g, h, nullptr);
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No path found");
  path_pub_->publish(path);
  return path;
}
