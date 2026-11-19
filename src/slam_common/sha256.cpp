#include "slam_common/sha256.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace slam_common {
namespace {

constexpr std::array<uint32_t, 64> kSha256K = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

inline uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32u - n)); }

std::string lower_ext(const std::filesystem::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

}  // namespace

Sha256::Sha256()
    : state_{0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
             0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u} {}

void Sha256::update(const uint8_t* data, size_t len) {
    if (buf_len_ > 0) {
        size_t take = std::min<size_t>(64 - buf_len_, len);
        std::memcpy(buf_ + buf_len_, data, take);
        buf_len_ += take;
        data += take;
        len -= take;
        if (buf_len_ == 64) {
            transform();
            bit_len_ += 512;
            buf_len_ = 0;
        }
    }
    while (len >= 64) {
        std::memcpy(buf_, data, 64);
        transform();
        bit_len_ += 512;
        data += 64;
        len -= 64;
    }
    if (len > 0) {
        std::memcpy(buf_, data, len);
        buf_len_ = len;
    }
}

void Sha256::update(const std::string& data) {
    update(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

std::string Sha256::final_hex() {
    uint64_t total_bits = bit_len_ + static_cast<uint64_t>(buf_len_) * 8ULL;
    buf_[buf_len_++] = 0x80;
    if (buf_len_ > 56) {
        while (buf_len_ < 64) buf_[buf_len_++] = 0;
        transform();
        buf_len_ = 0;
    }
    while (buf_len_ < 56) buf_[buf_len_++] = 0;
    for (int i = 7; i >= 0; --i) {
        buf_[buf_len_++] = static_cast<uint8_t>((total_bits >> (i * 8)) & 0xffu);
    }
    transform();

    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(64, '0');
    for (size_t i = 0; i < 8; ++i) {
        uint32_t v = state_[i];
        for (int b = 0; b < 4; ++b) {
            uint8_t byte = static_cast<uint8_t>((v >> ((3 - b) * 8)) & 0xffu);
            out[i * 8 + b * 2] = kHex[byte >> 4];
            out[i * 8 + b * 2 + 1] = kHex[byte & 0x0f];
        }
    }
    return out;
}

void Sha256::transform() {
    uint32_t m[64];
    for (int i = 0; i < 16; ++i) {
        m[i] = (static_cast<uint32_t>(buf_[i * 4]) << 24) |
               (static_cast<uint32_t>(buf_[i * 4 + 1]) << 16) |
               (static_cast<uint32_t>(buf_[i * 4 + 2]) << 8) |
               static_cast<uint32_t>(buf_[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = rotr(m[i - 15], 7) ^ rotr(m[i - 15], 18) ^ (m[i - 15] >> 3);
        uint32_t s1 = rotr(m[i - 2], 17) ^ rotr(m[i - 2], 19) ^ (m[i - 2] >> 10);
        m[i] = m[i - 16] + s0 + m[i - 7] + s1;
    }
    uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t temp1 = h + S1 + ch + kSha256K[i] + m[i];
        uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t temp2 = S0 + maj;
        h = g; g = f; f = e; e = d + temp1;
        d = c; c = b; b = a; a = temp1 + temp2;
    }
    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
}

std::string sha256_hex(const uint8_t* data, size_t len) {
    Sha256 ctx;
    ctx.update(data, len);
    return ctx.final_hex();
}

std::string sha256_hex(const std::string& data) {
    return sha256_hex(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

std::string sha256_file(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for SHA-256: " + path.string());
    Sha256 ctx;
    std::array<char, 65536> buf{};
    while (f) {
        f.read(buf.data(), buf.size());
        std::streamsize n = f.gcount();
        if (n > 0) ctx.update(reinterpret_cast<const uint8_t*>(buf.data()), static_cast<size_t>(n));
    }
    return ctx.final_hex();
}

std::string compute_canonical_sha256(const std::filesystem::path& path) {
    static const std::set<std::string> kBinaryExts = {
        ".pdf", ".png", ".jpg", ".jpeg", ".gz", ".mkv", ".bin", ".bag",
        ".ttf", ".ttc", ".otf", ".exe", ".dll", ".cmd", ".bat", ".ps1"
    };
    if (kBinaryExts.count(lower_ext(path))) {
        return sha256_file(path);
    }
    // 单遍流式读取：同时累计原始与 CRLF→LF 规范化两路摘要；
    // 任一块含 NUL 即判定为二进制，返回原始摘要。
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file for canonical SHA-256: " + path.string());
    Sha256 raw_ctx, norm_ctx;
    std::array<char, 65536> buf{};
    bool has_nul = false;
    bool pending_cr = false;
    auto feed_norm = [&](char c) {
        norm_ctx.update(reinterpret_cast<const uint8_t*>(&c), 1);
    };
    while (f) {
        f.read(buf.data(), buf.size());
        std::streamsize n = f.gcount();
        for (std::streamsize i = 0; i < n; ++i) {
            char c = buf[static_cast<size_t>(i)];
            if (c == '\0') has_nul = true;
            raw_ctx.update(reinterpret_cast<const uint8_t*>(&c), 1);
            if (pending_cr) {
                pending_cr = false;
                if (c == '\n') {
                    // CRLF：丢弃 '\r'，'\n' 照常输出
                } else {
                    feed_norm('\r');
                }
            }
            if (c == '\r') {
                pending_cr = true;
            } else {
                feed_norm(c);
            }
        }
    }
    if (pending_cr) feed_norm('\r');
    return has_nul ? raw_ctx.final_hex() : norm_ctx.final_hex();
}

namespace {

/// Parse "<hex>  <relpath>" lines from a SHA256SUMS file; missing file yields an empty list.
std::vector<std::pair<std::string, std::string>> read_checksum_entries(const std::filesystem::path& sums_path) {
    std::vector<std::pair<std::string, std::string>> entries;
    std::ifstream f(sums_path, std::ios::binary);
    if (!f) return entries;
    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
            line.pop_back();
        }
        size_t first_non_ws = line.find_first_not_of(" \t");
        if (first_non_ws == std::string::npos) continue;
        std::string trimmed = line.substr(first_non_ws);
        size_t ws = trimmed.find_first_of(" \t");
        if (ws == std::string::npos) continue;
        std::string digest = trimmed.substr(0, ws);
        size_t rest_pos = trimmed.find_first_not_of(" \t", ws);
        if (rest_pos == std::string::npos) continue;
        std::string rel = trimmed.substr(rest_pos);
        if (!rel.empty() && rel.front() == '*') rel.erase(rel.begin());
        entries.emplace_back(rel, digest);
    }
    return entries;
}

bool is_checksum_source_dir_name(const std::string& name) {
    return name == "build" || name == "dist" || name == "dist-linux" || name == "__pycache__" ||
           name.rfind(".build", 0) == 0 || name.rfind(".venv", 0) == 0;
}

bool is_preserved_offline_entry(const std::string& rel) {
    return rel.rfind("images/", 0) == 0 || rel.rfind("examples/", 0) == 0 ||
           rel.rfind("source/", 0) == 0 || rel.rfind("calibration/reference_20260908/", 0) == 0;
}

}  // namespace

ChecksumReport verify_repo_checksums(const std::filesystem::path& repo_root) {
    std::filesystem::path sums_path = repo_root / "SHA256SUMS";
    if (!std::filesystem::is_regular_file(sums_path)) {
        throw std::runtime_error("Missing SHA256SUMS at " + sums_path.string());
    }
    ChecksumReport report;
    for (const auto& entry : read_checksum_entries(sums_path)) {
        const std::string& rel = entry.first;
        const std::string& expected = entry.second;
        std::filesystem::path target = repo_root / std::filesystem::u8path(rel);
        if (!std::filesystem::is_regular_file(target)) continue;
        report.checked_count += 1;
        std::string actual = compute_canonical_sha256(target);
        if (actual != expected) {
            report.mismatches.push_back({rel, expected, actual});
        }
    }
    return report;
}

int update_repo_checksums(const std::filesystem::path& repo_root) {
    std::filesystem::path sums_path = repo_root / "SHA256SUMS";
    std::map<std::string, std::string> old_entries;
    for (const auto& entry : read_checksum_entries(sums_path)) {
        old_entries[entry.first] = entry.second;
    }

    // On-disk files that must receive a fresh canonical digest.
    std::set<std::string> tracked;
    static const std::set<std::string> kRootExts = {".md", ".txt", ".json", ".sh", ".cmd", ".ps1"};
    for (const auto& entry : std::filesystem::directory_iterator(repo_root)) {
        if (!entry.is_regular_file()) continue;
        std::string fname = entry.path().filename().generic_u8string();
        if (fname == "SHA256SUMS") continue;
        if (kRootExts.count(lower_ext(entry.path()))) {
            tracked.insert(fname);
        }
    }
    static const std::vector<std::string> kScanDirs = {
        "cmake", "include", "src", "tests", "calibration", "software"};
    for (const auto& sub : kScanDirs) {
        std::filesystem::path base = repo_root / std::filesystem::u8path(sub);
        if (!std::filesystem::is_directory(base)) continue;
        for (auto it = std::filesystem::recursive_directory_iterator(base);
             it != std::filesystem::recursive_directory_iterator(); ++it) {
            const std::filesystem::path& p = it->path();
            if (it->is_directory()) {
                if (is_checksum_source_dir_name(p.filename().generic_u8string())) {
                    it.disable_recursion_pending();
                }
                continue;
            }
            if (!it->is_regular_file()) continue;
            if (lower_ext(p) == ".pyc") continue;
            tracked.insert(p.lexically_relative(repo_root).generic_u8string());
        }
    }
    // Existing entries whose files are still on disk (covers files outside the scanned trees).
    for (const auto& kv : old_entries) {
        if (std::filesystem::is_regular_file(repo_root / std::filesystem::u8path(kv.first))) {
            tracked.insert(kv.first);
        }
    }

    std::map<std::string, std::string> entries;
    for (const auto& rel : tracked) {
        entries[rel] = compute_canonical_sha256(repo_root / std::filesystem::u8path(rel));
    }
    // Offline bundle artifacts absent from git keep their recorded digest.
    for (const auto& kv : old_entries) {
        if (tracked.count(kv.first)) continue;
        if (is_preserved_offline_entry(kv.first) &&
            !std::filesystem::is_regular_file(repo_root / std::filesystem::u8path(kv.first))) {
            entries[kv.first] = kv.second;
        }
    }

    std::ostringstream out;
    for (const auto& kv : entries) {
        out << kv.second << "  " << kv.first << "\n";
    }
    std::string payload = out.str();
    std::ofstream f(sums_path, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("Cannot write SHA256SUMS: " + sums_path.string());
    f.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    return static_cast<int>(tracked.size());
}

int64_t integer_ns(const std::string& value) {
    size_t start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        throw std::invalid_argument("Invalid nanosecond timestamp: " + value);
    }
    size_t end = value.find_last_not_of(" \t\r\n");
    std::string s = value.substr(start, end - start + 1);
    if (!s.empty() && s.front() == '+') s.erase(s.begin());
    if (s.empty() || s.front() == '-') {
        throw std::invalid_argument("Invalid nanosecond timestamp: " + value);
    }
    size_t dot = s.find('.');
    std::string int_part = (dot == std::string::npos) ? s : s.substr(0, dot);
    std::string frac_part = (dot == std::string::npos) ? "" : s.substr(dot + 1);
    if (int_part.empty()) {
        throw std::invalid_argument("Invalid nanosecond timestamp: " + value);
    }
    for (unsigned char c : int_part) {
        if (!std::isdigit(c)) throw std::invalid_argument("Invalid nanosecond timestamp: " + value);
    }
    for (unsigned char c : frac_part) {
        if (c != '0') throw std::invalid_argument("Invalid nanosecond timestamp: " + value);
    }
    uint64_t acc = 0;
    constexpr uint64_t kMaxI64 = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
    for (unsigned char c : int_part) {
        uint64_t d = static_cast<uint64_t>(c - '0');
        if (acc > (kMaxI64 - d) / 10ULL) {
            throw std::invalid_argument("Nanosecond timestamp overflows int64: " + value);
        }
        acc = acc * 10ULL + d;
    }
    return static_cast<int64_t>(acc);
}

void strictly_increasing(const std::vector<int64_t>& values, const std::string& label) {
    if (values.size() < 2) {
        throw std::invalid_argument(label + ": need at least two strictly increasing timestamps");
    }
    for (size_t i = 1; i < values.size(); ++i) {
        if (values[i] <= values[i - 1]) {
            throw std::invalid_argument(label + ": need at least two strictly increasing timestamps");
        }
    }
}

}  // namespace slam_common
