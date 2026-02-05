cmake_minimum_required(VERSION 3.10)

project(nature LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

find_package(X11)
if (X11_FOUND)
  include_directories(${X11_INCLUDE_DIR})
else()
  add_definitions(-Dcimg_display=0)
endif()

find_package(OpenMP)
if (OpenMP_CXX_FOUND)
  set(OPENMP_LIB OpenMP::OpenMP_CXX)
endif()

add_executable(nature_path_manager_node
  src/planning/global/path_manager_node.cpp
  src/node/node_proxy.cpp
)

add_executable(nature_gps_to_enu_node
  src/planning/global/gps_to_enu_node.cpp
  src/node/node_proxy.cpp
  src/planning/global/coord_conversions/coord_conversions.cpp
  src/planning/global/coord_conversions/ellipsoid.cpp
  src/planning/global/coord_conversions/matrix.cpp
)

add_executable(nature_gps_spoof_node
  src/planning/global/gps_spoof_node.cpp
  src/node/node_proxy.cpp
)

add_executable(nature_perception_node
  src/perception/nature_perception_node.cpp
  src/perception/elevation_grid.cpp
  src/node/node_proxy.cpp
)

add_executable(nature_ftte_node
  src/perception/ftte/nature_ftte_node.cpp
  src/perception/ftte/voxel_grid.cpp
  src/perception/ftte/vehicle.cpp
  src/perception/ftte/matrix.cpp
  src/perception/ftte/moreland.cpp
  src/node/node_proxy.cpp
)

add_executable(nature_map_publisher_node
  src/perception/nature_map_publisher_node.cpp
  src/node/node_proxy.cpp
)

add_executable(nature_control_node
  src/node/node_proxy.cpp
  src/control/nature_control_node.cpp
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  src/control/tinyfiledialogs.c
)

add_executable(nature_local_planner_node
  src/planning/local/nature_local_planner_node.cpp
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
  src/planning/local/spline_plotter.cpp
  src/planning/local/pf_planner.cpp
  src/planning/local/marker_spline_plotter.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)

add_executable(nature_pf_planner_node
  src/planning/local/nature_pf_planner_node.cpp
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)

add_executable(nature_global_path_node
  src/planning/global/nature_global_path_node.cpp
  src/planning/global/astar.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)

add_executable(nature_sim_test_node
  src/simulation/nature_sim_test_node.cpp
  src/node/node_proxy.cpp
  src/node/clock_publisher.cpp
  src/perception/point_cloud_generator.cpp
)

add_executable(nature_bot_state_publisher_node
  src/control/nature_bot_state_publisher.cpp
  src/node/node_proxy.cpp
)

set(LIB_SOURCES
  src/control/pid_controller.cpp
  src/control/pure_pursuit_controller.cpp
  src/perception/elevation_grid.cpp
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
  src/planning/local/spline_plotter.cpp
  src/visualization/image_visualizer.cpp
)

add_library(nature ${LIB_SOURCES})

foreach(target_name
        nature_path_manager_node
        nature_gps_to_enu_node
        nature_gps_spoof_node
        nature_perception_node
        nature_ftte_node
        nature_map_publisher_node
        nature_control_node
        nature_local_planner_node
        nature_pf_planner_node
        nature_global_path_node
        nature_sim_test_node
        nature_bot_state_publisher_node
        nature)
  if (X11_FOUND)
    target_link_libraries(${target_name} ${X11_LIBRARIES})
  endif()
  if (OpenMP_CXX_FOUND)
    target_link_libraries(${target_name} ${OPENMP_LIB})
  endif()
endforeach()
