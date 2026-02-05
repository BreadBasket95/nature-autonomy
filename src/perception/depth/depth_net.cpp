//
// TensorRT depth completion implementation.
//
#include "nature/perception/depth/depth_net.h"

#include <algorithm>
#include <iostream>
#include <vector>

#ifdef NATURE_HAS_TENSORRT
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>
#endif

namespace nature {
namespace perception {
namespace depth {

namespace {

#ifdef NATURE_HAS_TENSORRT
class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

template <typename T>
struct TrtDeleter {
  void operator()(T *ptr) const {
    if (ptr) {
      ptr->destroy();
    }
  }
};

int64_t volume(const nvinfer1::Dims &dims) {
  int64_t v = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    v *= dims.d[i] > 0 ? dims.d[i] : 1;
  }
  return v;
}
#endif

} // namespace

struct DepthNet::Impl {
#ifdef NATURE_HAS_TENSORRT
  TrtLogger logger;
  std::unique_ptr<nvinfer1::IBuilder, TrtDeleter<nvinfer1::IBuilder>> builder;
  std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter<nvinfer1::INetworkDefinition>> network;
  std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter<nvinfer1::IBuilderConfig>> config;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter<nvinfer1::ICudaEngine>> engine;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter<nvinfer1::IExecutionContext>> context;
  std::vector<void *> bindings;
  int input_rgb_index = -1;
  int input_sparse_index = -1;
  int output_depth_index = -1;
  int output_uncert_index = -1;
  float *dense_depth_gpu = nullptr;
  float *uncertainty_gpu = nullptr;
  int output_width = 0;
  int output_height = 0;
#endif
};

DepthNet::DepthNet() : impl_(new Impl()) {}

DepthNet::~DepthNet() {
#ifdef NATURE_HAS_TENSORRT
  if (impl_) {
    if (impl_->dense_depth_gpu) {
      cudaFree(impl_->dense_depth_gpu);
    }
    if (impl_->uncertainty_gpu) {
      cudaFree(impl_->uncertainty_gpu);
    }
  }
#endif
}

bool DepthNet::load_engine(const std::string &onnx_path, bool fp16) {
#ifdef NATURE_HAS_TENSORRT
  if (!impl_) {
    return false;
  }

  impl_->builder.reset(nvinfer1::createInferBuilder(impl_->logger));
  if (!impl_->builder) {
    return false;
  }
  const uint32_t flags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  impl_->network.reset(impl_->builder->createNetworkV2(flags));
  if (!impl_->network) {
    return false;
  }

  auto parser = std::unique_ptr<nvonnxparser::IParser, TrtDeleter<nvonnxparser::IParser>>(
      nvonnxparser::createParser(*impl_->network, impl_->logger));
  if (!parser || !parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    std::cerr << "Failed to parse ONNX: " << onnx_path << std::endl;
    return false;
  }

  impl_->config.reset(impl_->builder->createBuilderConfig());
  if (!impl_->config) {
    return false;
  }
  impl_->config->setMaxWorkspaceSize(1ULL << 28);
  if (fp16 && impl_->builder->platformHasFastFp16()) {
    impl_->config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  impl_->engine.reset(impl_->builder->buildEngineWithConfig(*impl_->network, *impl_->config));
  if (!impl_->engine) {
    std::cerr << "Failed to build TensorRT engine." << std::endl;
    return false;
  }
  impl_->context.reset(impl_->engine->createExecutionContext());
  if (!impl_->context) {
    return false;
  }

  const int nb_bindings = impl_->engine->getNbBindings();
  impl_->bindings.assign(nb_bindings, nullptr);

  // Identify bindings. Assumes two inputs and two outputs.
  for (int i = 0; i < nb_bindings; ++i) {
    if (impl_->engine->bindingIsInput(i)) {
      if (impl_->input_rgb_index < 0) {
        impl_->input_rgb_index = i;
      } else if (impl_->input_sparse_index < 0) {
        impl_->input_sparse_index = i;
      }
    } else {
      if (impl_->output_depth_index < 0) {
        impl_->output_depth_index = i;
      } else if (impl_->output_uncert_index < 0) {
        impl_->output_uncert_index = i;
      }
    }
  }

  if (impl_->output_depth_index < 0) {
    std::cerr << "TensorRT outputs not found." << std::endl;
    return false;
  }

  const nvinfer1::Dims depth_dims = impl_->engine->getBindingDimensions(impl_->output_depth_index);
  const int64_t depth_count = volume(depth_dims);
  if (depth_count <= 0) {
    std::cerr << "Invalid depth output dimensions." << std::endl;
    return false;
  }

  if (depth_dims.nbDims >= 2) {
    impl_->output_height = depth_dims.d[depth_dims.nbDims - 2];
    impl_->output_width = depth_dims.d[depth_dims.nbDims - 1];
  }

  cudaMalloc(&impl_->dense_depth_gpu, static_cast<size_t>(depth_count) * sizeof(float));
  if (impl_->output_uncert_index >= 0) {
    const nvinfer1::Dims uncert_dims = impl_->engine->getBindingDimensions(impl_->output_uncert_index);
    const int64_t uncert_count = volume(uncert_dims);
    cudaMalloc(&impl_->uncertainty_gpu, static_cast<size_t>(uncert_count) * sizeof(float));
  }

  return true;
#else
  (void)onnx_path;
  (void)fp16;
  return false;
#endif
}

bool DepthNet::infer(float *rgb_gpu,
                     float *sparse_depth_gpu,
                     DepthResult &out,
                     cudaStream_t stream) {
#ifdef NATURE_HAS_TENSORRT
  if (!impl_ || !impl_->context || impl_->input_rgb_index < 0 || impl_->output_depth_index < 0) {
    return false;
  }

  impl_->bindings[impl_->input_rgb_index] = rgb_gpu;
  if (impl_->input_sparse_index >= 0) {
    impl_->bindings[impl_->input_sparse_index] = sparse_depth_gpu;
  }
  impl_->bindings[impl_->output_depth_index] = impl_->dense_depth_gpu;
  if (impl_->output_uncert_index >= 0) {
    impl_->bindings[impl_->output_uncert_index] = impl_->uncertainty_gpu;
  }

  if (!impl_->context->enqueueV2(impl_->bindings.data(), stream, nullptr)) {
    return false;
  }

  out.dense_depth_gpu = impl_->dense_depth_gpu;
  out.uncertainty_gpu = impl_->uncertainty_gpu;
  out.width = impl_->output_width;
  out.height = impl_->output_height;
  return true;
#else
  (void)rgb_gpu;
  (void)sparse_depth_gpu;
  (void)out;
  (void)stream;
  return false;
#endif
}

} // namespace depth
} // namespace perception
} // namespace nature
