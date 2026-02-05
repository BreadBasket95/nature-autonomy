/**
 * \file gps_to_enu_node.cpp
 * 
 * Node for reading in a list of waypoints, converting to UTM, and republishing in ENU
 * 
 * \author Chris Goodin
 *
 * \contact dwc2@cavs.msstate.edu
 * 
 * \date 1/21/2022
 */

// messaging includes
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"
// local includes
#include "nature/nature_utils.h"
#include "nature/planning/global/coord_conversions/coord_conversions.h"
// c++ includes
#include <fstream>

bool fix_rcvd = false;
float lat_rcvd = 0.0f;
float lon_rcvd = 0.0f;
float alt_rcvd = 0.0f;

/**
 * @brief Handle incoming GPS fixes.
 * @param rcv_fix Incoming NavSatFix message.
 * @details Captures the first fix to establish the local origin.
 */
void NavSatCallback(nature::msg::NavSatFixPtr rcv_fix){
    if (!fix_rcvd){
        lat_rcvd = rcv_fix->latitude;
        lon_rcvd = rcv_fix->longitude;
        alt_rcvd = rcv_fix->altitude;
    }
  fix_rcvd = true;
}

/**
 * @brief Entry point for the GPS-to-ENU waypoint converter.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 * @details Loads GPS waypoints, converts them to UTM and then ENU relative
 *          to the first received fix, and publishes a Path.
 */
int main(int argc, char **argv){

    auto n = nature::node::init_node(argc,argv,"gps_to_enu_node");
    auto path_pub = n->create_publisher<nature::msg::Path>("nature/enu_waypoints", 10);
    auto navsat_sub = n->create_subscription<nature::msg::NavSatFix>("/piksi_imu/navsatfix_best_fix", 10, NavSatCallback);


    std::vector<double> gps_waypoints_lat, gps_waypoints_lon;

    n->get_parameter("/gps_waypoints_lon", gps_waypoints_lon, std::vector<double>(0));
    n->get_parameter("/gps_waypoints_lat", gps_waypoints_lat, std::vector<double>(0));

    if (gps_waypoints_lat.size()!=gps_waypoints_lon.size()){
        std::cerr<<"ERROR! IN THE GPS TO ENU WP FILE, THE NUMBER OF LAT AND LON ENTRIES WAS NOT THE SAME. EXITING."<<std::endl;
        return 1;
    }

    std::vector< std::vector<double> > path;
    nature::coordinate_system::CoordinateConverter converter;
    std::vector<nature::coordinate_system::UTM> utm_waypoints;
    path.reserve(gps_waypoints_lat.size());
    utm_waypoints.reserve(gps_waypoints_lat.size());
    for (int i=0;i<gps_waypoints_lat.size();i++){
        nature::coordinate_system::LLA gps_wp;
        gps_wp.latitude = gps_waypoints_lat[i];
        gps_wp.longitude = gps_waypoints_lon[i];
        gps_wp.altitude = 100.0f; // approximate elevation for Starkville, MS
        nature::coordinate_system::UTM utm_wp = converter.LLA2UTM(gps_wp);
        utm_waypoints.push_back(utm_wp);
        std::vector<double> point;
        point.push_back(utm_wp.x);
        point.push_back(utm_wp.y);
        path.push_back(point);
    }

    nature::node::Rate loop_rate(10);

    int count = 0;
    float utm_north = 0.0f;
    float utm_east = 0.0f;
    while (nature::node::ok()) {
        if (fix_rcvd){

            if (count==0){
                // first time only 
                nature::coordinate_system::LLA gps_origin;
                gps_origin.latitude = lat_rcvd;
                gps_origin.longitude = lon_rcvd;
                gps_origin.altitude = alt_rcvd; // approximate elevation for Starkville, MS
                nature::coordinate_system::UTM utm_origin = converter.LLA2UTM(gps_origin);
                utm_east = utm_origin.x;
                utm_north = utm_origin.y;
                std::ofstream fout;
                fout.open("gps_convert_log.txt");
                fout<<"UTM Origin: ("<<utm_east<<", "<<utm_north<<")"<<std::endl;
                fout<<"Waypoints: "<<std::endl;
                for (int32_t i = 0; i < path.size(); i++){
                    fout<<"("<<path[i][0] - utm_east<<", "<< path[i][1] - utm_north<<")"<<std::endl;
                }
                fout.close();
            }

            nature::msg::Path path_msg;
            path_msg.header.frame_id = "odom";
            path_msg.poses.clear();
            path_msg.poses.reserve(path.size());
            for (int32_t i = 0; i < path.size(); i++){
                nature::msg::PoseStamped pose;
                pose.pose.position.x = path[i][0] - utm_east;
                pose.pose.position.y = path[i][1] - utm_north;
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                path_msg.poses.push_back(pose);
            } 
            path_msg.header.stamp = n->get_stamp();
            for (int i = 0; i < path_msg.poses.size(); i++){
                path_msg.poses[i].header = path_msg.header;
            }
            path_pub->publish(path_msg);
            count++;
        }
        n->spin_some();
        loop_rate.sleep();
    }
}
