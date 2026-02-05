//
// TensorRT depth completion implementation.
//
#include "nature/perception/depth/depth_net.h"

#include <algorithm>
#include <cctype>
#include <fstream>
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

std::string to_lower(const std::string &input) {
  std::string out;
  out.reserve(input.size());
  for (char c : input) {
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

bool contains_token(const std::string &haystack, const std::string &token) {
  return haystack.find(token) != std::string::npos;
}

bool read_binary(const std::string &path, std::vector<char> &out) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in.is_open()) {
    return false;
  }
  const std::streamsize size = in.tellg();
  if (size <= 0) {
    return false;
  }
  out.resize(static_cast<size_t>(size));
  in.seekg(0, std::ios::beg);
  if (!in.read(out.data(), size)) {
    return false;
  }
  return true;
}

bool write_binary(const std::string &path, const void *data, size_t size) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
  return out.good();
}
#endif

} // namespace

struct DepthNet::Impl {
#ifdef NATURE_HAS_TENSORRT
  TrtLogger logger;
  std::unique_ptr<nvinfer1::IBuilder, TrtDeleter<nvinfer1::IBuilder>> builder;
  std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter<nvinfer1::INetworkDefinition>> network;
  std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter<nvinfer1::IBuilderConfig>> config;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter<nvinfer1::IRuntime>> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter<nvinfer1::ICudaEngine>> engine;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter<nvinfer1::IExecutionContext>> context;
  std::vector<void *> bindings;
  int input_rgb_index = -1;
  int input_sparse_index = -1;
  int output_depth_index = -1;
  int output_uncert_index = -1;
  bool input_rgb_nhwc = false;
  bool input_sparse_nhwc = false;
  bool has_dynamic_shapes = false;
  nvinfer1::Dims input_rgb_dims{};
  nvinfer1::Dims input_sparse_dims{};
  bool output_dynamic = false;
  size_t output_depth_bytes = 0;
  size_t output_uncert_bytes = 0;
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

bool DepthNet::load_engine(const std::string &onnx_path,
                           const std::string &engine_path,
                           bool fp16) {
#ifdef NATURE_HAS_TENSORRT
  if (!impl_) {
    return false;
  }
  impl_->has_dynamic_shapes = false;
  impl_->input_rgb_nhwc = false;
  impl_->input_sparse_nhwc = false;
  impl_->output_dynamic = false;
#ifdef NATURE_HAS_TENSORRT
  if (impl_->dense_depth_gpu) {
    cudaFree(impl_->dense_depth_gpu);
    impl_->dense_depth_gpu = nullptr;
    impl_->output_depth_bytes = 0;
  }
  if (impl_->uncertainty_gpu) {
    cudaFree(impl_->uncertainty_gpu);
    impl_->uncertainty_gpu = nullptr;
    impl_->output_uncert_bytes = 0;
  }
#endif

  if (!engine_path.empty()) {
    std::vector<char> cached;
    if (read_binary(engine_path, cached)) {
      impl_->runtime.reset(nvinfer1::createInferRuntime(impl_->logger));
      if (impl_->runtime) {
        impl_->engine.reset(impl_->runtime->deserializeCudaEngine(cached.data(), cached.size(), nullptr));
        if (impl_->engine) {
          impl_->context.reset(impl_->engine->createExecutionContext());
        }
      }
    }
  }

  if (impl_->engine && impl_->context) {
    // Cached engine loaded successfully.
  } else {
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

    if (!engine_path.empty()) {
      auto serialized = std::unique_ptr<nvinfer1::IHostMemory, TrtDeleter<nvinfer1::IHostMemory>>(
          impl_->engine->serialize());
      if (serialized) {
        if (!write_binary(engine_path, serialized->data(), serialized->size())) {
          std::cerr << "Failed to write TensorRT engine cache: " << engine_path << std::endl;
        }
      }
    }
  }

  const int nb_bindings = impl_->engine->getNbBindings();
  impl_->bindings.assign(nb_bindings, nullptr);

  // Identify bindings. Assumes two inputs and two outputs.
  for (int i = 0; i < nb_bindings; ++i) {
    if (impl_->engine->bindingIsInput(i)) {
      const std::string name = to_lower(impl_->engine->getBindingName(i));
      if ((contains_token(name, "rgb") || contains_token(name, "image") || contains_token(name, "color")) &&
          impl_->input_rgb_index < 0) {
        impl_->input_rgb_index = i;
      } else if ((contains_token(name, "sparse") || contains_token(name, "lidar") || contains_token(name, "depth")) &&
                 impl_->input_sparse_index < 0) {
        impl_->input_sparse_index = i;
      }
    } else {
      const std::string name = to_lower(impl_->engine->getBindingName(i));
      if ((contains_token(name, "uncert") || contains_token(name, "sigma")) && impl_->output_uncert_index < 0) {
        impl_->output_uncert_index = i;
      } else if (contains_token(name, "depth") && impl_->output_depth_index < 0) {
        impl_->output_depth_index = i;
      }
    }
  }

  // Fallback: pick first two inputs/outputs when names are unknown.
  if (impl_->input_rgb_index < 0 || impl_->input_sparse_index < 0) {
    for (int i = 0; i < nb_bindings; ++i) {
      if (!impl_->engine->bindingIsInput(i)) {
        continue;
      }
      if (impl_->input_rgb_index < 0) {
        impl_->input_rgb_index = i;
      } else if (impl_->input_sparse_index < 0) {
        impl_->input_sparse_index = i;
      }
    }
  }

  if (impl_->output_depth_index < 0) {
    // Choose the largest output tensor as depth.
    int64_t best_volume = -1;
    for (int i = 0; i < nb_bindings; ++i) {
      if (impl_->engine->bindingIsInput(i)) {
        continue;
      }
      const int64_t v = volume(impl_->engine->getBindingDimensions(i));
      if (v > best_volume) {
        best_volume = v;
        impl_->output_depth_index = i;
      }
    }
  }

  if (impl_->output_uncert_index < 0) {
    for (int i = 0; i < nb_bindings; ++i) {
      if (impl_->engine->bindingIsInput(i)) {
        continue;
      }
      if (i != impl_->output_depth_index) {
        impl_->output_uncert_index = i;
        break;
      }
    }
  }

  if (impl_->output_depth_index < 0) {
    std::cerr << "TensorRT outputs not found." << std::endl;
    return false;
  }

  if (impl_->input_rgb_index >= 0) {
    impl_->input_rgb_dims = impl_->engine->getBindingDimensions(impl_->input_rgb_index);
    impl_->has_dynamic_shapes = impl_->has_dynamic_shapes ||
                                std::any_of(impl_->input_rgb_dims.d,
                                            impl_->input_rgb_dims.d + impl_->input_rgb_dims.nbDims,
                                            [](int dim) { return dim < 0; });
    if (impl_->input_rgb_dims.nbDims == 4) {
      if (impl_->input_rgb_dims.d[1] == 3) {
        impl_->input_rgb_nhwc = false;
      } else if (impl_->input_rgb_dims.d[3] == 3) {
        impl_->input_rgb_nhwc = true;
      } else {
        // TODO: Handle non-standard RGB input layouts.
        impl_->input_rgb_nhwc = false;
      }
    }
  }

  if (impl_->input_sparse_index >= 0) {
    impl_->input_sparse_dims = impl_->engine->getBindingDimensions(impl_->input_sparse_index);
    impl_->has_dynamic_shapes = impl_->has_dynamic_shapes ||
                                std::any_of(impl_->input_sparse_dims.d,
                                            impl_->input_sparse_dims.d + impl_->input_sparse_dims.nbDims,
                                            [](int dim) { return dim < 0; });
    if (impl_->input_sparse_dims.nbDims == 4) {
      if (impl_->input_sparse_dims.d[1] == 1) {
        impl_->input_sparse_nhwc = false;
      } else if (impl_->input_sparse_dims.d[3] == 1) {
        impl_->input_sparse_nhwc = true;
      } else {
        // TODO: Handle non-standard sparse depth input layouts.
        impl_->input_sparse_nhwc = false;
      }
    }
  }

  const nvinfer1::Dims depth_dims = impl_->engine->getBindingDimensions(impl_->output_depth_index);
  impl_->output_dynamic = std::any_of(depth_dims.d,
                                      depth_dims.d + depth_dims.nbDims,
                                      [](int dim) { return dim < 0; });
  const int64_t depth_count = volume(depth_dims);
  if (depth_count <= 0) {
    std::cerr << "Invalid depth output dimensions." << std::endl;
    return false;
  }

  if (depth_dims.nbDims >= 2) {
    impl_->output_height = depth_dims.d[depth_dims.nbDims - 2];
    impl_->output_width = depth_dims.d[depth_dims.nbDims - 1];
  }

  if (!impl_->output_dynamic) {
    impl_->output_depth_bytes = static_cast<size_t>(depth_count) * sizeof(float);
    cudaMalloc(&impl_->dense_depth_gpu, impl_->output_depth_bytes);
  }
  if (impl_->output_uncert_index >= 0) {
    const nvinfer1::Dims uncert_dims = impl_->engine->getBindingDimensions(impl_->output_uncert_index);
    if (!impl_->output_dynamic &&
        !std::any_of(uncert_dims.d, uncert_dims.d + uncert_dims.nbDims, [](int dim) { return dim < 0; })) {
      const int64_t uncert_count = volume(uncert_dims);
      impl_->output_uncert_bytes = static_cast<size_t>(uncert_count) * sizeof(float);
      cudaMalloc(&impl_->uncertainty_gpu, impl_->output_uncert_bytes);
    } else {
      impl_->output_dynamic = true;
    }
  }

  return true;
#else
  (void)onnx_path;
  (void)fp16;
  return false;
#endif
}

bool DepthNet::load_engine(const std::string &onnx_path, bool fp16) {
  return load_engine(onnx_path, std::string(), fp16);
}

bool DepthNet::infer(float *rgb_gpu,
                     float *sparse_depth_gpu,
                     int width,
                     int height,
                     DepthResult &out,
                     cudaStream_t stream) {
#ifdef NATURE_HAS_TENSORRT
  if (!impl_ || !impl_->context || impl_->input_rgb_index < 0 || impl_->output_depth_index < 0) {
    return false;
  }

  if (impl_->has_dynamic_shapes) {
    auto set_input_dims = [&](int index, bool nhwc, int channels) -> bool {
      if (index < 0) {
        return true;
      }
      nvinfer1::Dims dims = impl_->engine->getBindingDimensions(index);
      if (dims.nbDims != 4) {
        // TODO: Support dynamic shapes for non-4D inputs.
        return false;
      }
      if (nhwc) {
        dims.d[0] = 1;
        dims.d[1] = height;
        dims.d[2] = width;
        dims.d[3] = channels;
      } else {
        dims.d[0] = 1;
        dims.d[1] = channels;
        dims.d[2] = height;
        dims.d[3] = width;
      }
      return impl_->context->setBindingDimensions(index, dims);
    };

    if (!set_input_dims(impl_->input_rgb_index, impl_->input_rgb_nhwc, 3)) {
      std::cerr << "Failed to set RGB binding dimensions." << std::endl;
      return false;
    }
    if (!set_input_dims(impl_->input_sparse_index, impl_->input_sparse_nhwc, 1)) {
      std::cerr << "Failed to set sparse depth binding dimensions." << std::endl;
      return false;
    }
    if (!impl_->context->allInputDimensionsSpecified()) {
      // TODO: Handle models that require optimization profiles or additional inputs.
      std::cerr << "TensorRT input dimensions not fully specified." << std::endl;
      return false;
    }
  }

  nvinfer1::Dims depth_dims = impl_->context->getBindingDimensions(impl_->output_depth_index);
  if (std::any_of(depth_dims.d, depth_dims.d + depth_dims.nbDims, [](int dim) { return dim < 0; })) {
    // TODO: Handle models with unresolved dynamic output dimensions.
    return false;
  }
  const int64_t depth_count = volume(depth_dims);
  const size_t depth_bytes = static_cast<size_t>(depth_count) * sizeof(float);
  if (!impl_->dense_depth_gpu || (impl_->output_dynamic && depth_bytes != impl_->output_depth_bytes)) {
    if (impl_->dense_depth_gpu) {
      cudaFree(impl_->dense_depth_gpu);
    }
    cudaMalloc(&impl_->dense_depth_gpu, depth_bytes);
    impl_->output_depth_bytes = depth_bytes;
  }
  if (depth_dims.nbDims >= 2) {
    impl_->output_height = depth_dims.d[depth_dims.nbDims - 2];
    impl_->output_width = depth_dims.d[depth_dims.nbDims - 1];
  }

  if (impl_->output_uncert_index >= 0) {
    const nvinfer1::Dims uncert_dims = impl_->context->getBindingDimensions(impl_->output_uncert_index);
    if (std::any_of(uncert_dims.d, uncert_dims.d + uncert_dims.nbDims, [](int dim) { return dim < 0; })) {
      // TODO: Handle dynamic uncertainty output dimensions.
    } else {
    const int64_t uncert_count = volume(uncert_dims);
    const size_t uncert_bytes = static_cast<size_t>(uncert_count) * sizeof(float);
    if (!impl_->uncertainty_gpu || (impl_->output_dynamic && uncert_bytes != impl_->output_uncert_bytes)) {
      if (impl_->uncertainty_gpu) {
        cudaFree(impl_->uncertainty_gpu);
      }
      cudaMalloc(&impl_->uncertainty_gpu, uncert_bytes);
      impl_->output_uncert_bytes = uncert_bytes;
    }
    }
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
  out.width = impl_->output_width > 0 ? impl_->output_width : width;
  out.height = impl_->output_height > 0 ? impl_->output_height : height;
  return true;
#else
  (void)rgb_gpu;
  (void)sparse_depth_gpu;
  (void)out;
  (void)stream;
  return false;
#endif
}

bool DepthNet::expects_rgb_nhwc() const {
#ifdef NATURE_HAS_TENSORRT
  return impl_ ? impl_->input_rgb_nhwc : false;
#else
  return false;
#endif
}

bool DepthNet::expects_sparse_nhwc() const {
#ifdef NATURE_HAS_TENSORRT
  return impl_ ? impl_->input_sparse_nhwc : false;
#else
  return false;
#endif
}

} // namespace depth
} // namespace perception
} // namespace nature
