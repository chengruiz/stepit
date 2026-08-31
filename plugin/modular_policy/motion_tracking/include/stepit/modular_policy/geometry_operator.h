#ifndef STEPIT_MODULAR_POLICY_GEOMETRY_OPERATOR_H_
#define STEPIT_MODULAR_POLICY_GEOMETRY_OPERATOR_H_

#include <stepit/field/operator.h>

namespace stepit {
namespace field {
class QuatRotateOperator : public Operator {
 public:
  explicit QuatRotateOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId quat_id_{};
  FieldId target_id_{};
  FieldSize source_size_{};
  std::size_t num_vectors_{};
  std::size_t num_quats_{};
  bool inverse_{false};
  ArrXf buffer_;
};

class QuatRotateBetweenOperator : public Operator {
 public:
  explicit QuatRotateBetweenOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId from_quat_id_{};
  FieldId to_quat_id_{};
  FieldId target_id_{};
  FieldSize source_size_{};
  std::size_t num_vectors_{};
  std::size_t num_from_quats_{};
  std::size_t num_to_quats_{};
  ArrXf buffer_;
};

class QuatInverseOperator : public Operator {
 public:
  explicit QuatInverseOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::size_t num_quats_{};
  ArrXf buffer_;
};

class QuatToEulerOperator : public Operator {
 public:
  explicit QuatToEulerOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::size_t num_quats_{};
  ArrXf buffer_;
};

class EulerToQuatOperator : public Operator {
 public:
  explicit EulerToQuatOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::size_t num_rpys_{};
  ArrXf buffer_;
};

class QuatToAngleAxisOperator : public Operator {
 public:
  explicit QuatToAngleAxisOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  std::size_t num_quats_{};
  ArrXf buffer_;
};

class QuatToRotation6dOperator : public Operator {
 public:
  explicit QuatToRotation6dOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  Rotation6dOrder order_{Rotation6dOrder::kRowMajor};
  std::size_t num_quats_{};
  ArrXf buffer_;
};
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_GEOMETRY_OPERATOR_H_
