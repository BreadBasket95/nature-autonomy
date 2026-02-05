# VIO + Depth Completion Perception Stack

**Role:** Senior C++ Embedded Perception Architect
**Objective:** Maintain and extend a Monocular VIO + Depth Completion perception stack inside the existing `nature-autonomy` (non-ROS) C++ framework.
**Target Hardware:** NVIDIA Jetson Orin (CUDA + TensorRT required).
**Framework Context:** This stack is a ROS-free, raw C++ autonomy system using `nature::msg` placeholder messages and `nature::node::NodeProxy`. It primarily uses 2D/2.5D occupancy grids in ENU/world coordinates. The system is ground-based today; your task is to adapt the perception layer for a 3D flight vehicle (quadcopter) while keeping outputs compatible with existing 2D/2.5D grid structures.

---

## 0) Constraints and Non-Negotiables

- **No ROS dependencies**. Do not include or link to `roscpp`, `rclcpp`, `catkin`, `ament`, etc.
- **Use existing messaging style** in this repo:
  - `include/nature/messaging/placeholder_messages.h`
  - `include/nature/messaging/message_types.h`
  - `nature::node::NodeProxy` for pubs/subs and parameter access.
- **C++14** (matches existing `CMakeLists.txt`).
- **Keep compatibility with existing topics** where possible (`nature/occupancy_grid`, `nature/odometry`) so planning/control nodes can run unchanged.
- **Do not modify third-party code** in `include/nature/thirdparty/` or `tinyfiledialogs.*`.
- **Prefer library-level integration** with OpenVINS (`ov_core`, `ov_msckf`) – do not use any ROS wrappers.
- **Performance** must be suitable for Jetson Orin. Use CUDA/TensorRT FP16 inference.

---

## 0.1) Transform Convention (CRITICAL)

All transforms in this codebase follow the convention:

**`T_ab` transforms a point from frame B to frame A**: `p_a = T_ab * p_b`

| Transform | Meaning |
|-----------|---------|
| `T_wb` | World → Body: transforms points from body frame to world frame |
| `T_bc` | Body → Camera: transforms points from camera frame to body frame |
| `T_cw` | Camera → World: transforms points from world frame to camera frame |

**Chaining example:** `T_cw = T_cb * T_bw = T_bc.inverse() * T_wb.inverse()`

Or equivalently for the forward direction: `T_cw = T_bc * T_wb` when `T_bc` represents body-to-camera and `T_wb` represents world-to-body.

**OpenVINS outputs:**
- `R_GtoI`: Rotation from Global (world) to IMU (body)
- `p_IinG`: Position of IMU origin expressed in Global frame

To construct `T_wb` (world→body transform):
```cpp
T_wb.block<3,3>(0,0) = R_GtoI;
T_wb.block<3,1>(0,3) = -R_GtoI * p_IinG;
```

---

## 0.2) Memory Management Requirements

For real-time operation on Jetson Orin:

1. **Avoid per-frame GPU allocations** in the processing loop. Use persistent buffers with growth factors (e.g., 1.5x) to reduce reallocations.
2. **Use CUDA streams** for async operations where possible.
3. **Prefer pinned host memory** for frequent host↔device transfers.
4. **RAII for GPU resources**: Use custom deleters or wrapper classes.

---

## 0.3) Temporal Integration Policy

The occupancy grid currently operates in **single-frame mode** (cleared each update). Future work may add:
- **Log-odds accumulation** for probabilistic occupancy over time
- **Decay/forgetting** for dynamic obstacles
- **Multi-resolution grids** for efficiency

If implementing temporal integration, ensure thread-safety and configurable decay rates.

---

## 1) Current Implementation Status

### Implemented Modules:

| Module | Location | Status |
|--------|----------|--------|
| VIOEstimator | `include/nature/perception/vio/vio_estimator.h`, `src/perception/vio/vio_estimator.cpp` | ✅ Complete (OpenVINS wrapper with async thread) |
| FeatureProjector | `include/nature/perception/depth/feature_projector.h`, `src/perception/depth/feature_projection.cu` | ✅ Complete (persistent GPU buffers, CPU fallback) |
| DepthNet | `include/nature/perception/depth/depth_net.h`, `src/perception/depth/depth_net.cpp` | ✅ Complete (TensorRT FP16, engine caching, dynamic shapes) |
| OccupancyMapper | `include/nature/perception/mapping/occupancy_mapper.h`, `src/perception/mapping/occupancy_mapper.cpp`, `occupancy_mapper_gpu.cu` | ✅ Complete (GPU+CPU paths with Bresenham raycasting) |
| vio_depth_node | `src/perception/vio_depth_node.cpp` | ✅ Complete (time sync, multi-threaded pipeline) |
| ZMQ Transport | `include/nature/transport/`, `src/transport/` | ✅ Complete (optional ICD pub/sub) |

### Known TODOs in Codebase:

1. **Sparse depth NHWC reordering** (`vio_depth_node.cpp:588`) - Handle networks expecting NHWC sparse input
2. **TensorRT optimization profiles** (`depth_net.cpp`) - Add profiles for dynamic shape performance
3. **Out-of-order measurement handling** (`vio_depth_node.cpp:89,103`) - Currently drops; could insert sorted
4. **Large IMU gaps** (`vio_depth_node.cpp:164`) - Add prediction or sensor fault reporting
5. **Camera origin outside grid** (`occupancy_mapper_gpu.cu:126`) - Handle edge case for raycasting

---

## 2) System Architecture: VIO + Depth Completion + Occupancy Mapping

### Overall Dataflow

```
IMU (200-400Hz) ──┐
                  ├──► TimeSyncBuffer ──► VIOEstimator ──► VIOUpdate
Camera (20-30Hz) ─┘                                            │
                                                               ▼
                                              FeatureProjector (CUDA)
                                                        │ sparse depth
                                                        ▼
                                              DepthNet (TensorRT FP16)
                                                        │ dense depth + uncertainty
                                                        ▼
                                              OccupancyMapper (CUDA/CPU)
                                                        │
                                                        ▼
                                              nature::msg::OccupancyGrid
                                                        │
                                        ┌───────────────┴───────────────┐
                                        ▼                               ▼
                              NodeProxy publish              ZMQ ICD serialize
                            "nature/occupancy_grid"          "icd/occupancy_grid"
```

### Coordinate Frames

- **World**: ENU (East, North, Up), consistent with the rest of the stack.
- **Body**: quadcopter body frame (IMU frame).
- **Camera**: camera optical frame (Z forward, X right, Y down).

---

## 3) Module APIs

### VIOEstimator (`nature::perception::vio`)

```cpp
struct VIOUpdate {
  double timestamp;
  Eigen::Matrix4f T_wb;              // World → Body transform
  std::vector<Eigen::Vector3f> landmarks_w; // Sparse landmarks in world frame
};

class VIOEstimator {
public:
  explicit VIOEstimator(const std::string& config_path);
  void feed_imu(double timestamp, const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro);
  void feed_image(double timestamp, const cv::Mat& image);
  void feed_measurement(double timestamp, const cv::Mat& image,
                        const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro);
  bool get_latest_update(VIOUpdate& out);
};
```

### FeatureProjector (`nature::perception::depth`)

```cpp
struct CameraIntrinsics {
  float fx, fy, cx, cy;
  int width, height;
};

// Preferred: persistent buffers
class FeatureProjector {
public:
  FeatureProjector();
  ~FeatureProjector();
  void generate_sparse_depth_map(const vio::VIOUpdate& vio,
                                 const Eigen::Matrix4f& T_bc,
                                 const CameraIntrinsics& K,
                                 float* gpu_output_buffer,
                                 cudaStream_t stream);
};

// Legacy: per-frame allocation (avoid in hot path)
void generate_sparse_depth_map(const vio::VIOUpdate& vio, ...);
```

### DepthNet (`nature::perception::depth`)

```cpp
struct DepthResult {
  float* dense_depth_gpu;
  float* uncertainty_gpu;  // May be nullptr
  int width, height;
};

class DepthNet {
public:
  bool load_engine(const std::string& onnx_path, const std::string& engine_path, bool fp16 = true);
  bool infer(float* rgb_gpu, float* sparse_depth_gpu, int width, int height,
             DepthResult& out, cudaStream_t stream);
  bool expects_rgb_nhwc() const;
  bool expects_sparse_nhwc() const;
};
```

### OccupancyMapper (`nature::perception::mapping`)

```cpp
class OccupancyMapper {
public:
  OccupancyMapper(float grid_res, float width, float height, float origin_x, float origin_y);
  void set_height_band(float min_z, float max_z);
  void set_uncertainty_threshold(float max_uncertainty);
  void set_use_gpu(bool enable);
  void update_from_depth(const DepthResult& depth, const Eigen::Matrix4f& T_wb,
                         const Eigen::Matrix4f& T_bc, const CameraIntrinsics& K);
  nature::msg::OccupancyGrid to_msg(bool row_major) const;
};
```

---

## 4) Configuration (`config/vio_depth.yaml`)

```yaml
# Camera intrinsics
camera.fx: 320.0
camera.fy: 320.0
camera.cx: 320.0
camera.cy: 240.0
camera.width: 640
camera.height: 480

# RGB normalization (for TensorRT input)
rgb.scale: 0.0039215686  # 1/255
rgb.mean: [0.0, 0.0, 0.0]
rgb.std: [1.0, 1.0, 1.0]

# Synthetic sensors (for testing without hardware)
synthetic_camera: true
synthetic_imu: true

# OpenVINS and TensorRT paths
openvins.config: config/openvins.yaml
tensorrt.onnx_path: models/iudc.onnx
tensorrt.engine_path: models/iudc.engine
tensorrt.fp16: true

# Time synchronization
sync.buffer_sec: 2.0
sync.time_slop: 0.02
sync.max_imu_gap: 0.05
sync.interpolate: true

# Body → Camera extrinsics (row-major 4x4)
t_bc: [1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0]

# Occupancy grid
grid.resolution: 0.5
grid.width: 200.0
grid.height: 200.0
grid.origin_x: -100.0
grid.origin_y: -100.0

# Height band filter (quadcopter collision volume)
height_band.min_z: -1.0
height_band.max_z: 3.0

# Uncertainty gating
uncertainty.max: 0.5

# Optional ZMQ transport
transport.zmq_enable: false
transport.zmq_pub_endpoint: tcp://*:5556
transport.zmq_sub_endpoint: tcp://localhost:5557
```

---

## 5) Build System

Enable with CMake options:
```bash
cmake -DNATURE_ENABLE_VIO_DEPTH=ON -DNATURE_ENABLE_ZMQ=ON ..
```

Creates targets:
- `nature_vio` (library)
- `nature_depth` (library, CUDA)
- `nature_mapping` (library, CUDA)
- `nature_transport` (library, optional)
- `nature_vio_depth_node` (executable)

---

## 6) Future Enhancement Opportunities

1. **Log-odds temporal integration** - Accumulate evidence over multiple frames
2. **Multi-resolution grids** - Coarse far-field, fine near-field
3. **Stereo/multi-camera support** - Extend VIOEstimator for stereo
4. **Dynamic obstacle detection** - Track moving objects separately
5. **GPU-accelerated VIO** - Move feature tracking to CUDA
6. **TensorRT INT8 quantization** - Further reduce inference latency
7. **Health monitoring** - Track VIO quality, depth confidence, sensor dropouts

---

## 7) Testing Guidance

- Build with `synthetic_camera: true` and `synthetic_imu: true` to test without hardware
- The pipeline produces valid (empty) occupancy grids with synthetic black frames
- Check TensorRT engine caching works (second run should be faster)
- Verify CPU fallback by building without CUDA defines

---

## 8) Acceptance Criteria for Changes

- Builds with CMake on Jetson Orin (CUDA/TensorRT)
- Produces valid `nature::msg::OccupancyGrid` compatible with existing planners
- No ROS dependencies
- Clean, maintainable C++14 style consistent with the repo
- Thread-safe and efficient (no per-frame GPU allocations in hot path)
- Graceful degradation when optional dependencies unavailable
