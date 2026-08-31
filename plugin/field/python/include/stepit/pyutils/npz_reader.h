#ifndef STEPIT_PYUTILS_NPZ_READER_H_
#define STEPIT_PYUTILS_NPZ_READER_H_

#include <string>

#include <stepit/field/data_loader.h>

namespace stepit {
namespace field {
class NpzReader : public DataLoader {
 public:
  explicit NpzReader(const std::string &path);
  NpzReader(const NpzReader &)            = delete;
  NpzReader &operator=(const NpzReader &) = delete;
  NpzReader(NpzReader &&)                 = default;
  NpzReader &operator=(NpzReader &&)      = default;
};
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_PYUTILS_NPZ_READER_H_
