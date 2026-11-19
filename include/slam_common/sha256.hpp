#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace slam_common {

/// Incremental SHA-256 hasher; final_hex() pads the stream and returns a lowercase hex digest.
class Sha256 {
public:
    Sha256();
    void update(const uint8_t* data, size_t len);
    void update(const std::string& data);
    std::string final_hex();

private:
    void transform();

    uint32_t state_[8];
    uint8_t buf_[64];
    size_t buf_len_ = 0;
    uint64_t bit_len_ = 0;
};

/// Compute lowercase hexadecimal SHA-256 digest of a raw byte buffer.
std::string sha256_hex(const uint8_t* data, size_t len);

/// Compute lowercase hexadecimal SHA-256 digest of a byte string.
std::string sha256_hex(const std::string& data);

/// Compute lowercase hexadecimal SHA-256 digest of a file on disk.
std::string sha256_file(const std::filesystem::path& path);

/// Compute canonical SHA-256 matching Linux sha256sum (normalizes CRLF->LF for text files).
std::string compute_canonical_sha256(const std::filesystem::path& path);

struct ChecksumMismatch {
    std::string rel_path;
    std::string expected;
    std::string actual;
};

struct ChecksumReport {
    int checked_count = 0;
    std::vector<ChecksumMismatch> mismatches;
};

/// Verify present repository files against <repo_root>/SHA256SUMS.
ChecksumReport verify_repo_checksums(const std::filesystem::path& repo_root);

/// Refresh <repo_root>/SHA256SUMS for all present files while preserving entries for unbundled offline archives.
int update_repo_checksums(const std::filesystem::path& repo_root);

/// Parse a non-negative nanosecond timestamp strictly without floating-point precision loss.
int64_t integer_ns(const std::string& value);

/// Ensure timestamp sequence contains >= 2 strictly increasing values.
void strictly_increasing(const std::vector<int64_t>& values, const std::string& label);

}  // namespace slam_common
