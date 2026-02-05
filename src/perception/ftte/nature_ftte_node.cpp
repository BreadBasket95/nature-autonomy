// c++ includes
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

// messaging includes
#include "nature/messaging/message_types.h"
#include "nature/messaging/point_cloud_conversion.h"
#include "nature/node/node_proxy.h"

// nature includes
#include "nature/perception/ftte/voxel_grid.h"

/**
 * @brief Convert a value to a zero-padded string.
 * @tparam T Type of the value.
 * @param x Value to convert.
 * @param zero_padding Minimum width with leading zeros.
 * @return Zero-padded string representation.
 * @details Used to generate consistent filenames for plot outputs.
 */
template <class T>
inline std::string ToString(T x, int zero_padding) {
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(zero_padding) << x;
  return ss.str();
}

nature::msg::Odometry current_pose;
bool odom_rcvd = false;
bool points_rcvd = false;
bool use_registered_points = false;
glm::vec3 current_position;
std::vector<glm::vec3> current_points;
bool using_loam = false;

/**
 * @brief Handle incoming point clouds for FTTE processing.
 * @param rcv_cloud Incoming point cloud message.
 * @details Converts the cloud, optionally transforms to world frame, and stores
 *          points for voxel grid processing.
 */
void PointCloudCallback(nature::msg::PointCloud2Ptr rcv_cloud) {
  nature::msg::PointCloud point_cloud;
  bool converted = nature::messaging::convertPointCloud2ToPointCloud(*rcv_cloud, point_cloud);
  if (odom_rcvd && converted) {
    current_points.clear();
    if (use_registered_points) {
      for (int p = 0; p < static_cast<int>(point_cloud.points.size()); p++) {
        if (using_loam) {
          current_points.push_back(glm::vec3(point_cloud.points[p].z, point_cloud.points[p].x, point_cloud.points[p].y));
        } else {
          current_points.push_back(glm::vec3(point_cloud.points[p].x, point_cloud.points[p].y, point_cloud.points[p].z));
        }
      }
    } else {
      nature::msg_tf::Quaternion q(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y,
                                   current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
      nature::msg_tf::Matrix3x3 R(q);
      nature::msg_tf::Vector3 origin(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y,
                                     current_pose.pose.pose.position.z);
      for (int p = 0; p < static_cast<int>(point_cloud.points.size()); p++) {
        nature::msg_tf::Vector3 v(point_cloud.points[p].x, point_cloud.points[p].y, point_cloud.points[p].z);
        nature::msg_tf::Vector3 vp = (R * v) + origin;
        current_points.push_back(glm::vec3(vp.x, vp.y, vp.z));
      }
    }
  }
  points_rcvd = true;
}

/**
 * @brief Handle incoming odometry updates.
 * @param rcv_odom Incoming odometry message.
 * @details Updates current pose and position; applies LOAM frame adjustments
 *          when configured.
 */
void OdometryCallback(nature::msg::OdometryPtr rcv_odom) {
  current_pose = *rcv_odom;
  if (using_loam) {
    current_pose.pose.pose.position.x = rcv_odom->pose.pose.position.z;
    current_pose.pose.pose.position.y = rcv_odom->pose.pose.position.x;
    current_pose.pose.pose.position.z = rcv_odom->pose.pose.position.y;
  }
  odom_rcvd = true;
  current_position = glm::vec3(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y,
                               current_pose.pose.pose.position.z);
}

/**
 * @brief Entry point for the FTTE traversability node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 * @details Configures the voxel grid, vehicle parameters, and visualization
 *          settings, then publishes traversability grids.
 */
int main(int argc, char *argv[]) {
  auto n = nature::node::init_node(argc, argv, "nature_ftte_node");

  auto pc_sub = n->create_subscription<nature::msg::PointCloud2>("nature/points", 10, PointCloudCallback);
  auto odom_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry", 10, OdometryCallback);
  auto grid_pub = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid", 10);
  auto grid_pub_vis = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid_vis", 10);

  n->get_parameter("~use_registered_points", use_registered_points, use_registered_points);
  n->get_parameter("~use_loam", using_loam, using_loam);

  float map_width = 150.0f;
  float map_length = 150.0f;
  float map_res = 1.0f;
  bool use_planes = false;
  bool show_timing = false;
  bool fixed_map = true;
  n->get_parameter("~map_res", map_res, map_res);
  n->get_parameter("~map_length", map_length, map_length);
  n->get_parameter("~map_width", map_width, map_width);
  n->get_parameter("~use_planes", use_planes, use_planes);
  n->get_parameter("~show_timing", show_timing, show_timing);
  n->get_parameter("~fixed_map", fixed_map, fixed_map);

  float vehicle_mass = 34251.0f;
  float vehicle_bumper_height = 0.41f;
  float vehicle_tire_radius = 0.4f;
  float vehicle_vci1 = 25.0f;
  float vehicle_max_slope = 0.55f;
  float vehicle_roof_height = 3.5f;
  n->get_parameter("~vehicle_mass", vehicle_mass, vehicle_mass);
  n->get_parameter("~vehicle_bumper_height", vehicle_bumper_height, vehicle_bumper_height);
  n->get_parameter("~vehicle_tire_radius", vehicle_tire_radius, vehicle_tire_radius);
  n->get_parameter("~vehicle_vci1", vehicle_vci1, vehicle_vci1);
  n->get_parameter("~vehicle_max_slope", vehicle_max_slope, vehicle_max_slope);
  n->get_parameter("~vehicle_roof_height", vehicle_roof_height, vehicle_roof_height);

  float default_traversability = 0.6f;
  n->get_parameter("~default_traversability", default_traversability, default_traversability);

  float slope_coeff = 0.25f;
  float slope_exponent = 2.0f;
  float soil_coeff = 0.0f;
  float soil_exponent = 1.0f;
  float veg_coeff = 1.0f;
  float veg_exponent = 1.0f;
  float roughness_coeff = 1.0f;
  float roughness_exponent = 1.0f;
  n->get_parameter("~slope_coeff", slope_coeff, slope_coeff);
  n->get_parameter("~slope_exponent", slope_exponent, slope_exponent);
  n->get_parameter("~soil_coeff", soil_coeff, soil_coeff);
  n->get_parameter("~soil_exponent", soil_exponent, soil_exponent);
  n->get_parameter("~veg_coeff", veg_coeff, veg_coeff);
  n->get_parameter("~veg_exponent", veg_exponent, veg_exponent);
  n->get_parameter("~roughness_coeff", roughness_coeff, roughness_coeff);
  n->get_parameter("~roughness_exponent", roughness_exponent, roughness_exponent);

  bool show_traversability = false;
  bool show_confidence = false;
  bool show_ground = false;
  bool show_roughness = false;
  bool show_veg = false;
  bool show_slope = false;
  n->get_parameter("~show_traversability", show_traversability, show_traversability);
  n->get_parameter("~show_confidence", show_confidence, show_confidence);
  n->get_parameter("~show_ground", show_ground, show_ground);
  n->get_parameter("~show_roughness", show_roughness, show_roughness);
  n->get_parameter("~show_veg", show_veg, show_veg);
  n->get_parameter("~show_slope", show_slope, show_slope);

  bool save_plots = false;
  n->get_parameter("~save_plots", save_plots, save_plots);

  double max_time = std::numeric_limits<double>::max();
  n->get_parameter("~max_time", max_time, max_time);

  // Create a vehicle for calculating traversability
  traverselib::Vehicle vehicle;
  vehicle.SetParams(vehicle_mass, vehicle_bumper_height, vehicle_tire_radius, vehicle_vci1, vehicle_max_slope,
                    vehicle_roof_height);
  vehicle.SetSlopeCoeff(slope_coeff);
  vehicle.SetSlopeExponent(slope_exponent);
  vehicle.SetSoilCoeff(soil_coeff);
  vehicle.SetSoilExponent(soil_exponent);
  vehicle.SetVegCoeff(veg_coeff);
  vehicle.SetVegExponent(veg_exponent);
  vehicle.SetRoughnessCoeff(roughness_coeff);
  vehicle.SetRoughnessExponent(roughness_exponent);

  // Create a traversability model
  traverselib::VoxelGrid grid;
  grid.SetDefaultTraversability(default_traversability);
  grid.SetAveragingRadius(3.0f * map_res);
  grid.UsePlaneFitting(use_planes);
  grid.SetPrintTimingInfo(show_timing);

  int frame_count = 0;
  double t0 = 0.0f;
  double elapsed_time = 0.0;
  std::ofstream fout("pose_log.txt");

  while (nature::node::ok() && elapsed_time < max_time) {
    if (odom_rcvd && points_rcvd) {
      if (frame_count == 0) {
        t0 = n->get_now_seconds();
      }
      if (!grid.Initialized()) {
        glm::vec3 llc(current_position.x - 0.5f * map_width, current_position.y - 0.5f * map_length,
                      current_position.z - 5.0);
        glm::vec3 urc(current_position.x + 0.5f * map_width, current_position.y + 0.5f * map_length,
                      current_position.z + 10.0);
        grid.Initialize(llc, urc, map_res);
        grid.SetVehicle(vehicle);
      } else if (!fixed_map) {
        glm::vec3 llc(current_position.x - 0.5f * map_width, current_position.y - 0.5f * map_length,
                      current_position.z - 5.0);
        glm::vec3 urc(current_position.x + 0.5f * map_width, current_position.y + 0.5f * map_length,
                      current_position.z + 10.0);
        grid.Move(llc, urc);
      }
      fout << elapsed_time << " " << current_position.x << " " << current_position.y << " " << current_position.z
           << std::endl;

      grid.AddRegisteredPoints(current_points, current_position);

      std::string file_prefix = ToString(frame_count, 5);

      if (show_confidence) {
        grid.PlotConfidence();
      }
      if (show_ground) {
        grid.PlotGround();
      }
      if (show_roughness) {
        grid.PlotRoughness();
      }
      if (show_veg) {
        grid.PlotVegDensity();
      }
      if (show_slope) {
        grid.PlotSlope();
      }
      if (show_traversability) {
        grid.PlotTraversability();
      }
      if (save_plots) {
        grid.SaveConfidencePlot(file_prefix + "_conf.bmp");
        grid.SaveGroundPlot(file_prefix + "_ground.bmp");
        grid.SaveRoughPlot(file_prefix + "_rough.bmp");
        grid.SaveVegDensityPlot(file_prefix + "_veg.bmp");
        grid.SaveSlopePlot(file_prefix + "_slope.bmp");
        grid.SaveTraversabilityPlot(file_prefix + "_trav.bmp");
      }

      nature::msg::OccupancyGrid occ_grid = grid.GetTraversabilityAsOccupancyGrid(false);
      occ_grid.header = current_pose.header;
      grid_pub->publish(occ_grid);

      nature::msg::OccupancyGrid occ_grid_vis = grid.GetTraversabilityAsOccupancyGrid(true);
      occ_grid_vis.header = current_pose.header;
      grid_pub_vis->publish(occ_grid_vis);

      points_rcvd = false;
      frame_count++;
      elapsed_time = n->get_now_seconds() - t0;
    }
    n->spin_some();
  }
  fout.close();
  return 0;
}
