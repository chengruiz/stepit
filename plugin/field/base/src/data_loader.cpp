#include <algorithm>

#include <stepit/field/data_loader.h>

namespace stepit {
namespace field {
NdArray::~NdArray() {
  if (ptr != nullptr) delete[] ptr;
}

NdArray::NdArray(NdArray &&other) noexcept
    : ptr(other.ptr),
      shape(std::move(other.shape)),
      nbytes(other.nbytes),
      dtype(std::move(other.dtype)),
      itemsize(other.itemsize) {
  other.ptr      = nullptr;
  other.nbytes   = 0;
  other.itemsize = 0;
}

NdArray &NdArray::operator=(NdArray &&other) noexcept {
  if (this != &other) {
    if (ptr != nullptr) delete[] ptr;
    ptr            = other.ptr;
    shape          = std::move(other.shape);
    nbytes         = other.nbytes;
    dtype          = std::move(other.dtype);
    itemsize       = other.itemsize;
    other.ptr      = nullptr;
    other.nbytes   = 0;
    other.itemsize = 0;
  }
  return *this;
}

void DataLoader::clear() {
  keys_.clear();
  values_.clear();
}

void DataLoader::insert(const std::string &key, NdArray array) {
  STEPIT_ASSERT(not hasKey(key), "Key '{}' already exists.", key);
  keys_.push_back(key);
  values_.push_back(std::move(array));
}

bool DataLoader::hasKey(const std::string &key) const {
  return std::find(keys_.begin(), keys_.end(), key) != keys_.end();
}

const NdArray &DataLoader::operator[](const std::string &key) const {
  auto it = std::find(keys_.begin(), keys_.end(), key);
  STEPIT_ASSERT(it != keys_.end(), "Key '{}' not found.", key);
  return values_[std::distance(keys_.begin(), it)];
}

CsvReader::CsvReader(const std::string &path) {
  fs::path input_dir(path);
  STEPIT_ASSERT(fs::is_directory(input_dir), "CSV path '{}' is not a directory.", path);

  auto metadata_path = input_dir / "metadata.yml";
  if (fs::is_regular_file(metadata_path)) {
    auto metadata = yml::loadFile(metadata_path.string());
    metadata["skip_header"].to(skip_header_, true);
  }

  std::vector<fs::path> csv_files;
  for (const auto &entry : fs::directory_iterator(input_dir)) {
    if (fs::is_regular_file(entry.path()) and entry.path().extension() == ".csv") {
      csv_files.push_back(entry.path());
    }
  }

  std::sort(csv_files.begin(), csv_files.end(),
            [](const fs::path &lhs, const fs::path &rhs) { return lhs.string() < rhs.string(); });
  STEPIT_ASSERT(not csv_files.empty(), "No CSV files found in '{}'.", path);

  for (const auto &csv_file : csv_files) {
    insert(csv_file.stem().string(), readCsvFile(csv_file.string()));
  }
}

NdArray CsvReader::readCsvFile(const std::string &path) const {
  std::ifstream file(path);
  STEPIT_ASSERT(file.good(), "Failed to open CSV file '{}'.", path);

  std::vector<float> data;
  std::string line;
  std::size_t row_idx  = 0;
  std::size_t num_cols = 0;
  bool header_skipped  = false;

  while (std::getline(file, line)) {
    auto trimmed = trim(line);
    if (trimmed.empty() or trimmed[0] == '#') continue;
    if (skip_header_ and not header_skipped) {
      header_skipped = true;
      continue;
    }

    std::vector<float> row;
    std::size_t token_begin = 0;
    while (true) {
      auto token_end = trimmed.find(',', token_begin);
      auto token     = trim(trimmed.substr(token_begin, token_end - token_begin));
      STEPIT_ASSERT(not token.empty(), "Empty value found in CSV file '{}' at parsed row {}.", path, row_idx);
      try {
        row.push_back(std::stof(token));
      } catch (const std::exception &err) {
        STEPIT_THROW("Failed to parse CSV value '{}' in '{}' at parsed row {}: {}", token, path, row_idx, err.what());
      }

      if (token_end == std::string::npos) break;
      token_begin = token_end + 1;
    }

    if (num_cols == 0) {
      num_cols = row.size();
      STEPIT_ASSERT(num_cols > 0, "CSV file '{}' does not contain any numeric columns.", path);
    } else {
      STEPIT_ASSERT(row.size() == num_cols,
                    "Inconsistent CSV column count in '{}': expected {}, but got {} at parsed row {}.", path, num_cols,
                    row.size(), row_idx);
    }

    data.insert(data.end(), row.begin(), row.end());
    ++row_idx;
  }

  STEPIT_ASSERT(row_idx > 0, "CSV file '{}' does not contain any data rows.", path);

  NdArray result;
  result.shape    = {row_idx, num_cols};
  result.dtype    = "float32";
  result.itemsize = sizeof(float);
  result.nbytes   = data.size() * sizeof(float);
  result.ptr      = new char[result.nbytes];
  std::memcpy(result.ptr, data.data(), result.nbytes);
  return result;
}

STEPIT_REGISTER_DATALOADER(csv, kDefPriority, DataLoader::make<CsvReader>);
}  // namespace field
}  // namespace stepit
