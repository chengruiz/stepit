#include <numeric>

#include <stepit/policy_neuro/field_ops.h>

namespace stepit {
namespace neuro_policy {
FieldOps::FieldOps(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "field_ops")), config_(loadConfig(policy_spec)) {
  auto ops_node = config_["ops"] ? config_["ops"] : config_;
  STEPIT_ASSERT(ops_node.IsSequence(), "'field_ops.yml' must be a sequence or contain an 'ops' sequence.");

  for (const auto &node : ops_node) {
    STEPIT_ASSERT(node.IsMap(), "Each field op must be a map.");
    STEPIT_ASSERT(yml::hasValue(node, "type"), "Each field op must contain a 'type'.");
    Operation operation;
    operation.node = YAML::Clone(node);

    auto op_type = yml::readAs<std::string>(node["type"]);
    if (op_type == "affine") {
      operation.type = OpType::kAffine;

      if (node["field"]) {
        auto field_name     = yml::readAs<std::string>(node["field"]);
        operation.source_id = registerRequirement(field_name);
        operation.target_id = operation.source_id;
      } else {
        STEPIT_ASSERT(node["source"] and node["target"],
                      "Affine op must contain 'field' or both 'source' and 'target'.");
        auto source_name    = yml::readAs<std::string>(node["source"]);
        auto target_name    = yml::readAs<std::string>(node["target"]);
        operation.source_id = registerRequirement(source_name);
        operation.target_id = registerProvision(target_name, 0);
      }
      STEPIT_ASSERT(not(node["scale"] and node["std"]), "Cannot specify both 'scale' and 'std' in an affine op.");
      STEPIT_ASSERT(not(node["bias"] and node["mean"]), "Cannot specify both 'bias' and 'mean' in an affine op.");
    } else if (op_type == "concat") {
      operation.type = OpType::kConcat;
      STEPIT_ASSERT(node["target"] and node["sources"], "Concat op must contain 'target' and 'sources'.");
      STEPIT_ASSERT(node["sources"].IsSequence(), "'sources' in concat op must be a sequence.");
      for (const auto &source_node : node["sources"]) {
        operation.source_ids.push_back(registerRequirement(yml::readAs<std::string>(source_node)));
      }
      operation.target_id = registerProvision(yml::readAs<std::string>(node["target"]), 0);
    } else if (op_type == "copy") {
      operation.type = OpType::kCopy;
      STEPIT_ASSERT(node["source"] and node["target"], "Copy op must contain 'source' and 'target'.");
      auto source_name = yml::readAs<std::string>(node, "source");
      auto target_name = yml::readAs<std::string>(node, "target");
      STEPIT_ASSERT(source_name != target_name, "Source and target cannot be the same in a copy op.");
      operation.source_id = registerRequirement(source_name);
      operation.target_id = registerProvision(target_name, 0);
    } else if (op_type == "split") {
      operation.type = OpType::kSplit;
      STEPIT_ASSERT(node["source"] and node["targets"], "Split op must contain 'source' and 'targets'.");
      operation.source_id = registerRequirement(yml::readAs<std::string>(node, "source"));

      const auto targets_node = node["targets"];
      STEPIT_ASSERT(targets_node.IsSequence(), "'targets' in split op must be a sequence.");
      for (const auto &target_node : targets_node) {
        STEPIT_ASSERT(target_node.IsMap() and target_node["name"] and target_node["size"],
                      "Each split target must be a map containing keys 'name' and 'size'.");
        auto name = yml::readAs<std::string>(target_node, "name");
        auto size = yml::readAs<FieldSize>(target_node, "size");
        operation.target_ids.push_back(registerProvision(name, size));
        operation.segment_sizes.push_back(size);
      }
    } else if (op_type == "slice") {
      operation.type = OpType::kSlice;
      STEPIT_ASSERT(node["source"] and node["target"], "Slice op must contain 'source' and 'target'.");
      operation.source_id = registerRequirement(yml::readAs<std::string>(node, "source"));
      operation.target_id = registerProvision(yml::readAs<std::string>(node, "target"), 0);

      if (yml::hasValue(node, "indices")) {
        const auto indices_node = node["indices"];
        STEPIT_ASSERT(indices_node.IsSequence() and indices_node.size() > 0,
                      "'indices' in slice op must be a non-empty sequence.");
        for (const auto &index_node : indices_node) {
          operation.indices.push_back(yml::readAs<FieldSize>(index_node));
        }
      } else {
        auto start = yml::readAs<FieldSize>(node, "start");
        auto end   = yml::readAs<FieldSize>(node, "end");
        STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) is invalid.", start, end);
        for (FieldSize i{start}; i < end; ++i) operation.indices.push_back(i);
      }
    } else {
      STEPIT_THROW("Unsupported field op type '{}'.", op_type);
    }

    operations_.push_back(std::move(operation));
  }
}

void FieldOps::initFieldProperties() {
  for (auto &operation : operations_) {
    switch (operation.type) {
      case OpType::kAffine: {
        auto field_size  = getFieldSize(operation.source_id);
        const auto &node = operation.node;
        operation.scale  = ArrXf::Ones(field_size);
        operation.bias   = ArrXf::Zero(field_size);

        if (yml::hasValue(node, "scale")) {
          yml::setTo(node, "scale", operation.scale);
        } else if (yml::hasValue(node, "std")) {
          ArrXf std{ArrXf::Ones(field_size)};
          yml::setTo(node, "std", std);
          STEPIT_ASSERT((std > kEPS).all(), "'std' values of affine op must be positive.");
          operation.scale = std.cwiseInverse();
        }
        if (yml::hasValue(node, "bias")) {
          yml::setTo(node, "bias", operation.bias);
        } else if (yml::hasValue(node, "mean")) {
          ArrXf mean{ArrXf::Zero(field_size)};
          yml::setTo(node, "mean", mean);
          operation.bias = -mean.cwiseProduct(operation.scale);
        }

        if (operation.target_id != operation.source_id) {
          setFieldSize(operation.target_id, field_size);
        }
        break;
      }
      case OpType::kConcat: {
        FieldSize total_size = 0;
        for (auto source_id : operation.source_ids) {
          auto source_size = getFieldSize(source_id);
          total_size += source_size;
        }
        setFieldSize(operation.target_id, total_size);
        operation.buffer.resize(total_size);
        break;
      }
      case OpType::kSplit: {
        auto source_size     = getFieldSize(operation.source_id);
        FieldSize total_size = std::accumulate(operation.segment_sizes.begin(), operation.segment_sizes.end(),
                                               static_cast<FieldSize>(0));
        STEPIT_ASSERT(total_size == source_size, "Split sizes ({}) do not match source size ({}) for '{}'.", total_size,
                      source_size, getFieldName(operation.source_id));
        break;
      }
      case OpType::kSlice: {
        auto source_size = getFieldSize(operation.source_id);
        for (auto index : operation.indices) {
          STEPIT_ASSERT(index < source_size, "Slice index {} is out of range [0, {}) for '{}'.", index, source_size,
                        getFieldName(operation.source_id));
        }
        auto target_size = static_cast<FieldSize>(operation.indices.size());
        setFieldSize(operation.target_id, target_size);
        operation.buffer.resize(target_size);
        break;
      }
      case OpType::kCopy: {
        auto source_size = getFieldSize(operation.source_id);
        setFieldSize(operation.target_id, source_size);
        break;
      }
      default:
        STEPIT_THROW("Unknown field op type.");
    }
  }
}

bool FieldOps::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (auto &operation : operations_) {
    switch (operation.type) {
      case OpType::kAffine: {
        auto transformed             = context.at(operation.source_id).cwiseProduct(operation.scale) + operation.bias;
        context[operation.target_id] = std::move(transformed);
        break;
      }
      case OpType::kConcat: {
        concatFields(context, operation.source_ids, operation.buffer);
        context[operation.target_id] = operation.buffer;
        break;
      }
      case OpType::kSplit: {
        splitFields(context.at(operation.source_id), operation.target_ids, context);
        break;
      }
      case OpType::kSlice: {
        const auto &source = context.at(operation.source_id);
        for (std::size_t i{}; i < operation.indices.size(); ++i) {
          operation.buffer[static_cast<Eigen::Index>(i)] = source[operation.indices[i]];
        }
        context[operation.target_id] = operation.buffer;
        break;
      }
      case OpType::kCopy: {
        context[operation.target_id] = context.at(operation.source_id);
        break;
      }
      default:
        STEPIT_THROW("Unknown field op type.");
    }
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_ops, kDefPriority, Module::make<FieldOps>);
}  // namespace neuro_policy
}  // namespace stepit
