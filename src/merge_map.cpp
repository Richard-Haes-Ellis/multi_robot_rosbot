
#include <thread>

#include <merge_map/merge_map.h>
#include <rcpputils/asserts.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

MapMerge::MapMerge()
    : Node("map_merge")
{

  // Create a publisher for the merged map
  // publisher_1 =
  //     this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_1",
  //                                                          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // publisher_2 =
  //     this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_2",
  //                                                          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  merged_map_publisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("merged_map",
                                                           rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create subscribers for the individual maps
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable();
  subscriber1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("rosbot1/map",map_qos,std::bind(&MapMerge::mapCallback, this, std::placeholders::_1));

  // // Create subscribers for the individual maps
  subscriber2_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("rosbot2/map",map_qos,std::bind(&MapMerge::mapCallback, this, std::placeholders::_1));

  // Timers
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]()
      { timerCallback(); });

  // Initialize the map
  old_map_msg.header.frame_id = "map";
  old_map_msg.info.resolution = 0.01;
  old_map_msg.info.width = 500;
  old_map_msg.info.height = 500;
  old_map_msg.info.origin.position.x = -2.5;
  old_map_msg.info.origin.position.y = -2.5;

  old_map_msg.data.resize(old_map_msg.info.width * old_map_msg.info.height);
  for (unsigned int i = 0; i < old_map_msg.info.height*old_map_msg.info.width; ++i)
  {
    old_map_msg.data[i] = -1; // Initially all unknown
  }
}

void MapMerge::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map_msg)
{
  
  nav_msgs::msg::OccupancyGrid tmp_map_msg;
  
  //  Orig: The frame origin
  //  00: The (0,0) point in our maps
  //  end: The end of axis in our maps
  //  map1: The incoming map
  //  map2: The map we currently have

  // mapa temporal
  tmp_map_msg.header.frame_id = "map";
  tmp_map_msg.info.resolution = new_map_msg->info.resolution;

  // Distance from origin of frame to origin of map in meters (NEW MAP)
  float dist_orig_00x_map1 = abs(new_map_msg->info.origin.position.x);
  float dist_orig_00y_map1 = abs(new_map_msg->info.origin.position.y);

  // Distance from origin of frame to end of map in meters (NEW MAP)
  float dist_orig_end_x_map1 = abs(new_map_msg->info.width  * new_map_msg->info.resolution - dist_orig_00x_map1);
  float dist_orig_end_y_map1 = abs(new_map_msg->info.height * new_map_msg->info.resolution - dist_orig_00y_map1);

  // Distance from origin of frame to origin of map in meters (OLD MAP)
  float dist_orig_00x_map2 = abs(old_map_msg.info.origin.position.x);
  float dist_orig_00y_map2 = abs(old_map_msg.info.origin.position.y);

  // Distance from origin of frame to end of map in meters (OLD MAP)
  float dist_orig_end_x_map2 = abs(old_map_msg.info.width  * old_map_msg.info.resolution - dist_orig_00x_map2);
  float dist_orig_end_y_map2 = abs(old_map_msg.info.height * old_map_msg.info.resolution - dist_orig_00y_map2);

  unsigned int start_map1_index_x = 0;
  unsigned int start_map1_index_y = 0;

  unsigned int start_map2_index_x = 0;
  unsigned int start_map2_index_y = 0;

  // Buscamos el origen del frame en el nuevo mapa
  if (dist_orig_00x_map1 > dist_orig_00x_map2)
  { 
    tmp_map_msg.info.origin.position.x = new_map_msg->info.origin.position.x;

    start_map1_index_x = 0;
    start_map2_index_x = static_cast<int>((dist_orig_00x_map1 - dist_orig_00x_map2) / tmp_map_msg.info.resolution);
  }
  else
  { 
    tmp_map_msg.info.origin.position.x = old_map_msg.info.origin.position.x;

    start_map1_index_x = static_cast<int>((dist_orig_00x_map2 - dist_orig_00x_map1) / tmp_map_msg.info.resolution);
    start_map2_index_x = 0;
  }

  if (dist_orig_00y_map1 > dist_orig_00y_map2)
  { 
    tmp_map_msg.info.origin.position.y = new_map_msg->info.origin.position.y;
    
    start_map1_index_y = 0;
    start_map2_index_y = static_cast<int>((dist_orig_00y_map1 - dist_orig_00y_map2) / tmp_map_msg.info.resolution);
  }
  else
  { 
    tmp_map_msg.info.origin.position.y = old_map_msg.info.origin.position.y;
    
    start_map1_index_y = static_cast<int>((dist_orig_00y_map2 - dist_orig_00y_map1) / tmp_map_msg.info.resolution);
    start_map2_index_y = 0;
  }


  // Actualizamos el ancho acorde al solapamiento del nuevo mapa
  if (dist_orig_end_x_map1 > dist_orig_end_x_map2)
  {
    tmp_map_msg.info.width = static_cast<int>((dist_orig_end_x_map1 + abs(tmp_map_msg.info.origin.position.x)) / tmp_map_msg.info.resolution);
  }
  else
  {
    tmp_map_msg.info.width = static_cast<int>((dist_orig_end_x_map2 + abs(tmp_map_msg.info.origin.position.x)) / tmp_map_msg.info.resolution);
  }

  // Lo mmismo pero con el alto
  if (dist_orig_end_y_map1 > dist_orig_end_y_map2)
  {
    tmp_map_msg.info.height = static_cast<int>((dist_orig_end_y_map1 + abs(tmp_map_msg.info.origin.position.y)) / tmp_map_msg.info.resolution);
  }
  else
  {
    tmp_map_msg.info.height = static_cast<int>((dist_orig_end_y_map2 + abs(tmp_map_msg.info.origin.position.y)) / tmp_map_msg.info.resolution);
  }

  tmp_map_msg.data.resize(tmp_map_msg.info.width * tmp_map_msg.info.height);

  // La magia de fusionar datos
  for (unsigned int y = 0; y < tmp_map_msg.info.height; ++y)
  {
    for (unsigned int x = 0; x < tmp_map_msg.info.width; ++x)
    {
      // If range is in map1 and map2
      if ((x >= start_map2_index_x && x < (start_map2_index_x + old_map_msg.info.width) && y >= start_map2_index_y && y < (start_map2_index_y + old_map_msg.info.height)) &&
          (x >= start_map1_index_x && x < (start_map1_index_x + new_map_msg->info.width) && y >= start_map1_index_y && y < (start_map1_index_y + new_map_msg->info.height)))
      {
        char occupancy_map1 = new_map_msg->data[(y - start_map1_index_y) * new_map_msg->info.width + (x - start_map1_index_x)];
        char occupancy_map2 = old_map_msg.data[(y - start_map2_index_y) * old_map_msg.info.width + (x - start_map2_index_x)];

        // The most probable case is that the new map is more likely to be occupied than the old one
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = (occupancy_map1 > occupancy_map2) ? occupancy_map1 : occupancy_map2;
      }
      else if (x >= start_map2_index_x && x < (start_map2_index_x + old_map_msg.info.width) && y >= start_map2_index_y && y < (start_map2_index_y + old_map_msg.info.height))
      {
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = old_map_msg.data[(y - start_map2_index_y) * old_map_msg.info.width + (x - start_map2_index_x)];
      }
      else if (x >= start_map1_index_x && x < (start_map1_index_x + new_map_msg->info.width) && y >= start_map1_index_y && y < (start_map1_index_y + new_map_msg->info.height))
      {
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = new_map_msg->data[(y - start_map1_index_y) * new_map_msg->info.width + (x - start_map1_index_x)];
      }else{
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = -1;
      }
    }
  }

  auto now = this->now();
  old_map_msg.header.stamp = now;
  old_map_msg.info.width = tmp_map_msg.info.width;
  old_map_msg.info.height = tmp_map_msg.info.height;
  old_map_msg.info.origin.position.x = tmp_map_msg.info.origin.position.x;
  old_map_msg.info.origin.position.y = tmp_map_msg.info.origin.position.y;

  old_map_msg.data.resize(tmp_map_msg.info.width * tmp_map_msg.info.height);

  for(unsigned int i = 0; i < tmp_map_msg.info.width * tmp_map_msg.info.height; i++){
    old_map_msg.data[i] = tmp_map_msg.data[i];
  }

  merged_map_publisher_->publish(old_map_msg);
}

void MapMerge::timerCallback()
{
  /*
  // RCLCPP_INFO(this->get_logger(), "Received map");
  nav_msgs::msg::OccupancyGrid tmp_map_msg;
  // tmp_map_msg = *msg;

  
  //  Orig: The frame origin
  //  00: The (0,0) point in our maps
  //  end: The end of axis in our maps
  //  msg: The incoming map
  //  map: The map we currently have

  

  // mapa temporal
  tmp_map_msg.header.frame_id = "map";
  tmp_map_msg.info.resolution = mp1_map_msg.info.resolution;

  // Distance from origin of frame to origin of map in meters (NEW MAP)
  float dist_orig_00x_map1 = abs(mp1_map_msg.info.origin.position.x);
  float dist_orig_00y_map1 = abs(mp1_map_msg.info.origin.position.y);

  // Distance from origin of frame to end of map in meters (NEW MAP)
  float dist_orig_end_x_map1 = abs(mp1_map_msg.info.width  * mp1_map_msg.info.resolution - dist_orig_00x_map1);
  float dist_orig_end_y_map1 = abs(mp1_map_msg.info.height * mp1_map_msg.info.resolution - dist_orig_00y_map1);

  // Distance from origin of frame to origin of map in meters (OLD MAP)
  float dist_orig_00x_map2 = abs(mp2_map_msg.info.origin.position.x);
  float dist_orig_00y_map2 = abs(mp2_map_msg.info.origin.position.y);

  // Distance from origin of frame to end of map in meters (OLD MAP)
  float dist_orig_end_x_map2 = abs(mp2_map_msg.info.width  * mp2_map_msg.info.resolution - dist_orig_00x_map2);
  float dist_orig_end_y_map2 = abs(mp2_map_msg.info.height * mp2_map_msg.info.resolution - dist_orig_00y_map2);

  unsigned int start_map1_index_x = 0;
  unsigned int start_map1_index_y = 0;

  unsigned int start_map2_index_x = 0;
  unsigned int start_map2_index_y = 0;

  // Buscamos el origen del frame en el nuevo mapa
  if (dist_orig_00x_map1 > dist_orig_00x_map2)
  { 
    tmp_map_msg.info.origin.position.x = mp1_map_msg.info.origin.position.x;

    start_map1_index_x = 0;
    start_map2_index_x = static_cast<int>((dist_orig_00x_map1 - dist_orig_00x_map2) / tmp_map_msg.info.resolution);
  }
  else
  { 
    tmp_map_msg.info.origin.position.x = mp2_map_msg.info.origin.position.x;

    start_map1_index_x = static_cast<int>((dist_orig_00x_map2 - dist_orig_00x_map1) / tmp_map_msg.info.resolution);
    start_map2_index_x = 0;
  }

  if (dist_orig_00y_map1 > dist_orig_00y_map2)
  { 
    tmp_map_msg.info.origin.position.y = mp1_map_msg.info.origin.position.y;
    
    start_map1_index_y = 0;
    start_map2_index_y = static_cast<int>((dist_orig_00y_map1 - dist_orig_00y_map2) / tmp_map_msg.info.resolution);
  }
  else
  { 
    tmp_map_msg.info.origin.position.y = mp2_map_msg.info.origin.position.y;
    
    start_map1_index_y = static_cast<int>((dist_orig_00y_map2 - dist_orig_00y_map1) / tmp_map_msg.info.resolution);
    start_map2_index_y = 0;
  }


  // Actualizamos el ancho acorde al solapamiento del nuevo mapa
  if (dist_orig_end_x_map1 > dist_orig_end_x_map2)
  {
    tmp_map_msg.info.width = static_cast<int>((dist_orig_end_x_map1 + abs(tmp_map_msg.info.origin.position.x)) / tmp_map_msg.info.resolution);
  }
  else
  {
    tmp_map_msg.info.width = static_cast<int>((dist_orig_end_x_map2 + abs(tmp_map_msg.info.origin.position.x)) / tmp_map_msg.info.resolution);
  }

  // Lo mmismo pero con el alto
  if (dist_orig_end_y_map1 > dist_orig_end_y_map2)
  {
    tmp_map_msg.info.height = static_cast<int>((dist_orig_end_y_map1 + abs(tmp_map_msg.info.origin.position.y)) / tmp_map_msg.info.resolution);
  }
  else
  {
    tmp_map_msg.info.height = static_cast<int>((dist_orig_end_y_map2 + abs(tmp_map_msg.info.origin.position.y)) / tmp_map_msg.info.resolution);
  }

  tmp_map_msg.data.resize(tmp_map_msg.info.width * tmp_map_msg.info.height);

  RCLCPP_INFO(this->get_logger(), "Map1: Start x,y: %d, %d. W,H: %d, %d.", start_map1_index_x,start_map1_index_y,start_map1_index_x + mp1_map_msg.info.width, start_map1_index_y + mp1_map_msg.info.height);  
  RCLCPP_INFO(this->get_logger(), "Map2: Start x,y: %d, %d. W,H: %d, %d.", start_map2_index_x,start_map2_index_y,start_map2_index_x + mp2_map_msg.info.width, start_map2_index_y + mp2_map_msg.info.height);

  // La magia de fusionar datos
  for (unsigned int y = 0; y < tmp_map_msg.info.height; ++y)
  {
    for (unsigned int x = 0; x < tmp_map_msg.info.width; ++x)
    {
      // If range is in map1 and map2
      if ((x >= start_map2_index_x && x < (start_map2_index_x + mp2_map_msg.info.width) && y >= start_map2_index_y && y < (start_map2_index_y + mp2_map_msg.info.height)) &&
          (x >= start_map1_index_x && x < (start_map1_index_x + mp1_map_msg.info.width) && y >= start_map1_index_y && y < (start_map1_index_y + mp1_map_msg.info.height)))
      {
        char occupancy_map1 = mp1_map_msg.data[(y - start_map1_index_y) * mp1_map_msg.info.width + (x - start_map1_index_x)];
        char occupancy_map2 = mp2_map_msg.data[(y - start_map2_index_y) * mp2_map_msg.info.width + (x - start_map2_index_x)];

        // The most probable case is that the new map is more likely to be occupied than the old one
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = (occupancy_map1 > occupancy_map2) ? occupancy_map1 : occupancy_map2;
      }
      else if (x >= start_map2_index_x && x < (start_map2_index_x + mp2_map_msg.info.width) && y >= start_map2_index_y && y < (start_map2_index_y + mp2_map_msg.info.height))
      {
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = mp2_map_msg.data[(y - start_map2_index_y) * mp2_map_msg.info.width + (x - start_map2_index_x)];
      }
      else if (x >= start_map1_index_x && x < (start_map1_index_x + mp1_map_msg.info.width) && y >= start_map1_index_y && y < (start_map1_index_y + mp1_map_msg.info.height))
      {
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = mp1_map_msg.data[(y - start_map1_index_y) * mp1_map_msg.info.width + (x - start_map1_index_x)];
      }else{
        tmp_map_msg.data[y * tmp_map_msg.info.width + x] = -1;
      }
    }
  }

  auto now = this->now();
  mp1_map_msg.header.stamp = now;
  mp2_map_msg.header.stamp = now;
  tmp_map_msg.header.stamp = now;

  mp1_map_msg.info.map_load_time = now;
  mp2_map_msg.info.map_load_time = now;
  tmp_map_msg.info.map_load_time = now;

  publisher_3->publish(tmp_map_msg);
  publisher_2->publish(mp2_map_msg);
  publisher_1->publish(mp1_map_msg);

  */
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapMerge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
