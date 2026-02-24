#include <algorithm>
#include <iostream>
#include <sstream>

#include <stepit/logging.h>
#include <stepit/utils.h>
#include <stepit/nnrt/nnrt.h>

namespace stepit {
std::string shape2str(const std::vector<int64_t> &shape) {
  std::stringstream ss;
  ss << "[" << shape[0];
  for (int j{1}; j < shape.size(); ++j) ss << " x " << shape[j];
  ss << "]";
  return ss.str();
}

NnrtApi::NnrtApi(const std::string &path, const YAML::Node &config) : config_(config) {
  STEPIT_ASSERT(fs::exists(path), "Model path '{}' does not exist.", path);
  path_ = fs::canonical(path).string();
}

void NnrtApi::warmup(int iterations) {
  Timer timer;
  timer.start();
  for (int i{}; i < iterations; ++i) runInference();
  synchronize();
  timer.stop();
  STEPIT_DBUGNT("Average warmup inference time: {}.", timer.total<USec>() / iterations);
}

std::size_t NnrtApi::getInputIdx(const std::string &name, bool assert) const {
  auto it = std::find(in_names_.begin(), in_names_.end(), name);
  if (it == in_names_.end()) {
    if (not assert) return -1;
    STEPIT_THROW("No input named '{}'.", name);
  }
  return it - in_names_.begin();
}

std::size_t NnrtApi::getOutputIdx(const std::string &name, bool assert) const {
  auto it = std::find(out_names_.begin(), out_names_.end(), name);
  if (it == out_names_.end()) {
    if (not assert) return -1;
    STEPIT_THROW("No output named '{}'.", name);
  }
  return it - out_names_.begin();
}

void NnrtApi::postInit() {
  if (config_["input_name"]) {
    std::vector<std::string> in_names;
    yml::setTo(config_, "input_name", in_names);
    for (int i{}; i < num_in_; ++i) {
      if (in_names_[i] != in_names[i]) {
        STEPIT_WARNNT("Name of input {} mismatch ('{}' != '{}').", i, in_names[i], in_names_[i]);
        in_names_[i] = in_names[i];
      }
    }
  }

  if (config_["output_name"]) {
    std::vector<std::string> out_names;
    yml::setTo(config_, "output_name", out_names);
    for (int i{}; i < num_out_; ++i) {
      if (out_names_[i] != out_names[i]) {
        STEPIT_WARNNT("Name of output {} mismatch ('{}' != '{}').", i, out_names[i], out_names_[i]);
        out_names_[i] = out_names[i];
      }
    }
  }

  if (config_["recurrent_param"]) {
    recur_params_.clear();
    yml::setTo(config_, "recurrent_param", recur_params_);
  } else {
    for (const auto &in_name : in_names_) {
      if (in_name == "h0") {
        recur_params_.emplace_back("h0", "hn");
      } else if (in_name == "c0") {
        recur_params_.emplace_back("c0", "cn");
      } else {
        const std::string prefix = "memory_in";
        if (in_name.rfind(prefix, 0) != 0) continue;
        std::string suffix = in_name.substr(prefix.size());
        recur_params_.emplace_back(in_name, "memory_out" + suffix);
      }
    }
  }
  in_recur_.resize(num_in_, false);
  out_recur_.resize(num_out_, false);
  for (int i{}; i < recur_params_.size(); ++i) {
    std::size_t in_idx  = getInputIdx(recur_params_[i].first, true);
    std::size_t out_idx = getOutputIdx(recur_params_[i].second, true);
    in_recur_[in_idx]   = true;
    out_recur_[out_idx] = true;
    recur_param_indices_.emplace_back(in_idx, out_idx);
  }
}

void NnrtApi::printInfo() const {
  std::stringstream ss;
  ss << "Model: " << path_;
  for (int i{}; i < num_in_; ++i) {
    ss << fmt::format("\n- in  {} {}: {} = {}", i, in_names_[i], shape2str(in_shapes_[i]), in_sizes_[i]);
  }
  for (int i{}; i < num_out_; ++i) {
    ss << fmt::format("\n- out {} {}: {} = {}", i, out_names_[i], shape2str(out_shapes_[i]), out_sizes_[i]);
  }
  for (int i{}; i < recur_params_.size(); ++i) {
    ss << fmt::format("\n- rec {} (out {}) -> {} (in {})", recur_params_[i].second, recur_param_indices_[i].second,
                      recur_params_[i].first, recur_param_indices_[i].first);
  }
  std::cout << ss.str() << std::endl;
}

template class NnrtApi::Interface;
}  // namespace stepit
