//
// TensorRT-based depth completion wrapper.
//
#ifndef NATURE_DEPTH_NET_H
#define NATURE_DEPTH_NET_H

#include <memory>
#include <string>

// Provide a fallback stream type when CUDA headers are unavailable.
#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#else
typedef void *cudaStream_t;
#endif

namespace nature {
namespace perception {
namespace depth {

/**
 * @brief Depth inference outputs from the completion network.
 */
struct DepthResult {
  float *dense_depth_gpu = nullptr;
  float *uncertainty_gpu = nullptr;
  int width = 0;
  int height = 0;
};

/**
 * @brief Wrapper for a TensorRT depth completion network.
 * @details Builds an engine from ONNX and runs inference in FP16 when available.
 */
class DepthNet {
public:
  /**
   * @brief Construct the depth network wrapper.
   */
  DepthNet();

  /**
   * @brief Destroy the network and free GPU resources.
   */
  ~DepthNet();

  /**
   * @brief Load and build a TensorRT engine from an ONNX file.
   * @param onnx_path Path to the ONNX model.
   * @param fp16 Enable FP16 inference if supported.
   * @return True if the engine was built successfully.
   */
  bool load_engine(const std::string &onnx_path, bool fp16 = true);

  /**
   * @brief Run inference on GPU inputs.
   * @param rgb_gpu Pointer to RGB input on GPU (NCHW float).
   * @param sparse_depth_gpu Pointer to sparse depth on GPU (1xHxW float).
   * @param out Output buffers populated with GPU pointers and dimensions.
   * @param stream CUDA stream to use for execution.
   * @return True if inference was launched successfully.
   */
  bool infer(float *rgb_gpu,
             float *sparse_depth_gpu,
             DepthResult &out,
             cudaStream_t stream);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace depth
} // namespace perception
} // namespace nature

#endif // NATURE_DEPTH_NET_H
