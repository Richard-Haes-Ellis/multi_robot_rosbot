

#ifndef MERGE_MAP_H_
#define MERGE_MAP_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>


#include <geometry_msgs/msg/transform.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/thread.hpp>


class MapMerge : public rclcpp::Node
{

public:
  MapMerge();

private:

  // publishing
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_publisher_;
  
  // subscribing
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber1_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber2_;

  // timers
  rclcpp::TimerBase::SharedPtr timer_;

  // map update
  nav_msgs::msg::OccupancyGrid old_map_msg;


  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerCallback();

};




#endif /* MAP_MERGE_H_ */
