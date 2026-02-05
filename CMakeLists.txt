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

option(NATURE_ENABLE_VIO_DEPTH "Build VIO + depth completion pipeline" OFF)

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

if (NATURE_ENABLE_VIO_DEPTH)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(CUDA REQUIRED)
  enable_language(CUDA)

  set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  if (NOT EIGEN_INCLUDE_DIRS)
    set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  endif()

  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include /usr/local/include /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu)
  find_library(TENSORRT_LIB nvinfer
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)
  find_library(NVONNXPARSER_LIB nvonnxparser
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

  if (NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB OR NOT NVONNXPARSER_LIB)
    message(FATAL_ERROR "TensorRT not found. Set TENSORRT_INCLUDE_DIR/TENSORRT_LIB/NVONNXPARSER_LIB.")
  endif()

  find_path(OPENVINS_INCLUDE_DIR ov_msckf/VioManager.h
    PATHS /usr/local/include /usr/include)
  find_library(OPENVINS_CORE_LIB ov_core
    PATHS /usr/local/lib /usr/lib)
  find_library(OPENVINS_MSCKF_LIB ov_msckf
    PATHS /usr/local/lib /usr/lib)

  set(HAVE_OPENVINS FALSE)
  if (OPENVINS_INCLUDE_DIR AND OPENVINS_CORE_LIB AND OPENVINS_MSCKF_LIB)
    set(HAVE_OPENVINS TRUE)
  else()
    message(WARNING "OpenVINS not found; VIOEstimator will run in placeholder mode.")
  endif()

  add_library(nature_vio
    src/perception/vio/vio_estimator.cpp
  )
  target_include_directories(nature_vio PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
  )
  if (HAVE_OPENVINS)
    target_include_directories(nature_vio PUBLIC ${OPENVINS_INCLUDE_DIR})
    target_compile_definitions(nature_vio PRIVATE NATURE_HAS_OPENVINS)
    target_link_libraries(nature_vio ${OPENVINS_CORE_LIB} ${OPENVINS_MSCKF_LIB})
  endif()
  target_link_libraries(nature_vio ${OpenCV_LIBS})
  if (TARGET Eigen3::Eigen)
    target_link_libraries(nature_vio Eigen3::Eigen)
  endif()

  add_library(nature_depth
    src/perception/depth/depth_net.cpp
    src/perception/depth/feature_projection.cu
  )
  target_include_directories(nature_depth PUBLIC
    include
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
  )
  target_compile_definitions(nature_depth PRIVATE NATURE_HAS_CUDA NATURE_HAS_TENSORRT)
  target_link_libraries(nature_depth ${CUDA_LIBRARIES} ${TENSORRT_LIB} ${NVONNXPARSER_LIB})
  set_target_properties(nature_depth PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  set_target_properties(nature_depth PROPERTIES CUDA_STANDARD 14 CUDA_STANDARD_REQUIRED ON)

  add_library(nature_mapping
    src/perception/mapping/occupancy_mapper.cpp
  )
  target_include_directories(nature_mapping PUBLIC include ${EIGEN_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
  target_compile_definitions(nature_mapping PRIVATE NATURE_HAS_CUDA)
  target_link_libraries(nature_mapping ${CUDA_LIBRARIES})
  if (TARGET Eigen3::Eigen)
    target_link_libraries(nature_mapping Eigen3::Eigen)
  endif()

  add_executable(nature_vio_depth_node
    src/perception/vio_depth_node.cpp
    src/node/node_proxy.cpp
  )
  target_include_directories(nature_vio_depth_node PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
  )
  target_compile_definitions(nature_vio_depth_node PRIVATE NATURE_HAS_CUDA NATURE_HAS_TENSORRT)
  target_link_libraries(nature_vio_depth_node
    nature_vio
    nature_depth
    nature_mapping
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIB}
    ${NVONNXPARSER_LIB}
  )
  if (TARGET Eigen3::Eigen)
    target_link_libraries(nature_vio_depth_node Eigen3::Eigen)
  endif()
  if (X11_FOUND)
    target_link_libraries(nature_vio_depth_node ${X11_LIBRARIES})
  endif()
  if (OpenMP_CXX_FOUND)
    target_link_libraries(nature_vio_depth_node ${OPENMP_LIB})
  endif()
endif()
