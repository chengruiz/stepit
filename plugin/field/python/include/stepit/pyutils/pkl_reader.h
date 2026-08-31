#ifndef STEPIT_PYUTILS_PKL_READER_H_
#define STEPIT_PYUTILS_PKL_READER_H_

#include <string>

#include <stepit/field/data_loader.h>

namespace stepit {
namespace field {
class PklReader : public DataLoader {
 public:
  explicit PklReader(const std::string &path);
  PklReader(const PklReader &)            = delete;
  PklReader &operator=(const PklReader &) = delete;
  PklReader(PklReader &&)                 = default;
  PklReader &operator=(PklReader &&)      = default;
};
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_PYUTILS_PKL_READER_H_
