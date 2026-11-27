#pragma once

// Player identity for LAN play: a stable 64-bit id derived from the player's
// name, so a seat can be agreed on without trusting arrival order or "the host
// is player 0".

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace apex {

using PlayerId = std::uint64_t;

// 0 means "no player": an empty name hashes to 0, so callers can tell "nobody
// chose a name yet" apart from a real id without a second flag.
constexpr PlayerId kInvalidPlayerId = 0;

// The wire name field is 16 bytes including the terminator (kNameBytes in
// net/protocol/wire.h). The id hashes exactly the bytes that travel, so both
// sides derive the same id from the same packet.
constexpr std::size_t kPlayerNameBytes = 16;
constexpr std::size_t kPlayerNameMaxChars = kPlayerNameBytes - 1;

// Trimmed and capped to kPlayerNameMaxChars, spelling kept. This is the name
// that is displayed and that travels on the wire.
std::string TrimPlayerName(std::string_view name);

// TrimPlayerName lowercased. Ids hash this form, so "Pilot" and "  PILOT " are
// the same player while the display keeps the spelling that was typed.
std::string NormalizePlayerName(std::string_view name);

// FNV-1a 64 over the normalized name. `variant` (1, 2, …) is appended as "#<n>"
// before hashing: that is how a caller who already knows a name is taken
// rehashes to a different id deterministically. Rehashing is the only collision
// repair the protocol offers — the host refuses a duplicate id outright
// (RejectReason::DuplicatePeer), so a name clash costs a rename, never a car
// running on someone else's seat.
PlayerId MakePlayerId(std::string_view name, std::uint32_t variant = 0);

}  // namespace apex
