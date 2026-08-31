#ifndef STEPIT_MODULAR_POLICY_MOTION_CLIP_H_
#define STEPIT_MODULAR_POLICY_MOTION_CLIP_H_

#include <memory>
#include <string>
#include <vector>

#include <stepit/utils.h>
#include <stepit/field/data_loader.h>

namespace stepit {
namespace modular_policy {
class MotionClip {
 public:
  enum class DataType { kNumeric, kQuaternion };

  MotionClip(std::string path, const std::string &dataloader_factory);
  std::vector<ArrXf> loadField(const std::string &name, const std::string &key, yml::Indices indices,
                               const std::string &type, bool differentiate, double output_fps);

  const std::string &path() const { return path_; }

 private:
  static DataType parseDataType(const std::string &type);
  ArrXf interpolateFrame(const std::vector<ArrXf> &source, DataType data_type, double source_pos) const;
  std::vector<ArrXf> resampleFrames(const std::vector<ArrXf> &source, double output_fps, DataType data_type) const;
  std::vector<ArrXf> differentiateFrames(const std::vector<ArrXf> &source, double output_fps, DataType data_type) const;

  std::string path_;
  std::unique_ptr<field::DataLoader> data_;
  double fps_{};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_MOTION_CLIP_H_
