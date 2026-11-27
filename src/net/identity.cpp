#include "net/identity.h"

#include <cctype>

namespace apex {
namespace {

constexpr PlayerId kFnvOffset = 1469598103934665603ull * 10ull + 7ull;  // 14695981039346656037
constexpr PlayerId kFnvPrime = 1099511628211ull;

bool IsSpace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

}  // namespace

std::string TrimPlayerName(std::string_view name) {
    std::size_t begin = 0;
    std::size_t end = name.size();
    while (begin < end && IsSpace(name[begin])) ++begin;
    while (end > begin && IsSpace(name[end - 1])) --end;

    std::string out;
    out.reserve(end - begin);
    for (std::size_t i = begin; i < end && out.size() < kPlayerNameMaxChars; ++i) {
        out.push_back(name[i]);
    }
    return out;
}

std::string NormalizePlayerName(std::string_view name) {
    std::string out = TrimPlayerName(name);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

PlayerId MakePlayerId(std::string_view name, std::uint32_t variant) {
    std::string text = NormalizePlayerName(name);
    if (text.empty()) return kInvalidPlayerId;
    if (variant > 0) {
        text.push_back('#');
        text += std::to_string(variant);
    }
    PlayerId hash = kFnvOffset;
    for (char c : text) {
        hash ^= static_cast<PlayerId>(static_cast<unsigned char>(c));
        hash *= kFnvPrime;
    }
    // A hash of 0 would be indistinguishable from "no player"; the salt is a
    // pure function of the name, so re-deriving it on both ends stays stable.
    return hash == kInvalidPlayerId ? 1u : hash;
}

}  // namespace apex
