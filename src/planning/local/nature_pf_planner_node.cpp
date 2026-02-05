/**
 * \file nature_pf_planner_node.cpp
 * Plan a local trajectory using the potential field planner
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 1/19/2022
 */
// messaging includes
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"
// nature includes
#include "nature/planning/local/pf_planner.h"
#include "nature/visualization/visualization_factory.h"

nature::msg::Odometry odom;
nature::msg::OccupancyGrid grid;
nature::msg::OccupancyGrid segmentation_grid;
nature::msg::Path global_path;
nature::msg::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;
bool new_seg_grid_rcvd = false;

/**
 * @brief Handle incoming odometry updates.
 * @param rcv_odom Incoming odometry message.
 * @details Updates the vehicle state used for potential field planning.
 */
void OdometryCallback(nature::msg::OdometryPtr rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

/**
 * @brief Handle incoming occupancy grid updates.
 * @param rcv_grid Incoming occupancy grid.
 * @details Updates the obstacle grid used by the planner.
 */
void GridCallback(nature::msg::OccupancyGridPtr rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

/**
 * @brief Handle incoming segmentation grid updates.
 * @param rcv_grid Incoming segmentation grid.
 * @details Updates semantic grid used for terrain cost adjustments.
 */
void SegmentationGridCallback(nature::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
    new_seg_grid_rcvd = true;
}

/**
 * @brief Handle incoming global path updates.
 * @param rcv_path Incoming global path.
 * @details Stores the path used when configured to follow global endpoints.
 */
void PathCallback(nature::msg::PathPtr rcv_path){
  global_path = *rcv_path;
}

/**
 * @brief Handle incoming waypoint updates.
 * @param wp_path Incoming waypoint path.
 * @details Stores the waypoint list used to set local goals.
 */
void WaypointCallback(nature::msg::PathPtr wp_path){
  waypoints = *wp_path;
}

/**
 * @brief Entry point for the potential field local planner node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 * @details Configures planner gains, subscribes to grids and odometry,
 *          and publishes local paths computed by potential fields.
 */
int main(int argc, char *argv[]){

  auto n = nature::node::init_node(argc, argv, "nature_pf_planner_node");

  // Create publishers and subscribers
  auto path_pub = n->create_publisher<nature::msg::Path>("nature/local_path", 10);
  auto odometry_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry", 10, OdometryCallback);
  auto grid_sub = n->create_subscription<nature::msg::OccupancyGrid>("nature/occupancy_grid", 10, GridCallback);
  auto segmentation_grid_sub = n->create_subscription<nature::msg::OccupancyGrid>("nature/segmentation_grid", 10, SegmentationGridCallback);
  auto path_sub = n->create_subscription<nature::msg::Path>("nature/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription<nature::msg::Path>("nature/waypoints", 10, WaypointCallback);

  // planner params
  float kp, eta, cutoff_dist, inner_cutoff_dist, rate;
  bool use_global_path;
  int obs_cost_thresh;
  n->get_parameter("~kp", kp, 5.0f);
  n->get_parameter("~eta", eta, 100.0f);
  n->get_parameter("~obstacle_cost_thresh", obs_cost_thresh, 0);
  n->get_parameter("~cutoff_dist", cutoff_dist, 20.0f);
  n->get_parameter("~inner_cutoff_dist", inner_cutoff_dist, 1.5f);
  n->get_parameter("~use_global_path", use_global_path, false);
  n->get_parameter("~rate", rate, 50.0f);

  nature::planning::PfPlanner planner;
  planner.SetEta(eta);
  planner.SetKp(kp);
  planner.SetCutoffDistance(cutoff_dist);
  planner.SetInnerCutoff(inner_cutoff_dist);
  planner.SetObstacleCostThreshold(obs_cost_thresh);

  unsigned int loop_count = 0;
  float dt = 1.0f / rate;
  float elapsed_time = 0.0f;
  nature::node::Rate loop_rate(rate);
  while (nature::node::ok()){
    double start_secs = n->get_now_seconds();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){

      float gx, gy;
      if (use_global_path){
        gx = global_path.poses.back().pose.position.x;
        gy = global_path.poses.back().pose.position.y;
      }
      else{
        gx = waypoints.poses.back().pose.position.x;
        gy = waypoints.poses.back().pose.position.y;
      }

      planner.SetGoal(gx, gy);

      if (new_seg_grid_rcvd) planner.SetSegGrid(segmentation_grid);
      nature::msg::Path local_path = planner.Plan(grid, odom);

      local_path.header.frame_id = "odom";
      local_path.header.stamp = n->get_stamp();
      nature::node::set_seq(local_path.header, loop_count);
      path_pub->publish(local_path);
      /*}
      else {
        nature::msg::Path local_path;
        nature::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        nature::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
      }*/

      odom_rcvd = false;
    }
    else {
      if (global_path.poses.size() <= 0){
        //std::cout << "Local planner did not run because global path not recieved " << std::endl;
      }
      else if (!odom_rcvd){
        //std::cout << "Local planner did not run because vehicle odometry not recieved." << std::endl;
      }
      else if (grid.data.size() <= 0){
        //std::cout << "Local planner did not run because occupancy grid not recieved." << std::endl;
      }
    }
    new_grid_rcvd = false;
    new_seg_grid_rcvd = false;
    loop_count++;
    double end_secs = n->get_now_seconds();
    if ((end_secs - start_secs) > 2.5 * dt){
      std::cout << "WARNING: POTENTIAL FIELD PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    n->spin_some();
    loop_rate.sleep();
  }

  return 0;
}
