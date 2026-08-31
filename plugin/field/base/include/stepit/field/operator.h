#ifndef STEPIT_FIELD_OPERATOR_H_
#define STEPIT_FIELD_OPERATOR_H_

#include <stepit/field/field.h>

namespace stepit {
namespace field {
class Operator : public Node, public Interface<Operator, const yml::Node & /* config */> {
 public:
  /** Initializes field-dependent state and throws UndefinedFieldSpecError when an input specification is unresolved. */
  virtual void init() {}
  /** Clears persistent state before a policy run. */
  virtual bool reset() { return true; }
  /** Reads input fields and writes output fields for the current step. */
  virtual bool update(FieldMap &context) = 0;
  /** Updates delayed state after all modules have completed the current step. */
  virtual void postStep(const FieldMap &context) {}
};

class AffineOperator : public Operator {
 public:
  explicit AffineOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  yml::Node node_;
  FieldId source_id_{};
  FieldId target_id_{};
  FieldSize field_size_{};
  ArrXf scale_;
  ArrXf bias_;
};

class CastOperator : public Operator {
 public:
  explicit CastOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  DataType target_dtype_{DataType::kUndefined};
};

class ConcatOperator : public Operator {
 public:
  explicit ConcatOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldIdVec source_ids_;
  FieldId target_id_{};
};

class ConstOperator : public Operator {
 public:
  explicit ConstOperator(const yml::Node &config);

  bool update(FieldMap &context) override;

 private:
  FieldId target_id_{};
  ArrXf value_;
};

class CopyOperator : public Operator {
 public:
  explicit CopyOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
};

class HistoryOperator : public Operator {
 public:
  explicit HistoryOperator(const yml::Node &config);

  void init() override;
  bool reset() override;
  bool update(FieldMap &context) override;
  void postStep(const FieldMap &context) override;

 private:
  void push(const FieldValue &frame);
  void render(FieldValue &target) const;

  FieldId source_id_{};
  FieldId target_id_{};
  std::uint32_t history_len_{};
  FieldSize source_size_{};
  FieldSize target_size_{};
  yml::Indices indices_;
  bool newest_first_{true};
  bool include_current_frame_{true};
  bool has_default_value_{true};
  yml::Node default_value_node_;
  FieldValue default_value_;
  RingBuffer<FieldValue> history_;
};

class MaskedFillOperator : public Operator {
 public:
  explicit MaskedFillOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  FieldSize field_size_{};
  yml::Indices indices_;
  float value_{};
  ArrXf buffer_;
};

class SliceOperator : public Operator {
 public:
  explicit SliceOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldId target_id_{};
  yml::Indices indices_;
};

class SplitOperator : public Operator {
 public:
  explicit SplitOperator(const yml::Node &config);

  void init() override;
  bool update(FieldMap &context) override;

 private:
  FieldId source_id_{};
  FieldIdVec target_ids_;
  std::vector<FieldSize> segment_sizes_;
};
}  // namespace field
}  // namespace stepit

#define STEPIT_REGISTER_FIELD_OPERATOR(name, priority, factory) \
  static ::stepit::field::Operator::Registration _field_operation_##name##_registration(#name, priority, factory)

#endif  // STEPIT_FIELD_OPERATOR_H_
