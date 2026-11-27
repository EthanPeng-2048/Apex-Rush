#include "net/roster.h"

#include <algorithm>
#include <vector>

namespace apex::net {

Roster BuildRoster(std::span<const RosterCandidate> candidates) {
    std::vector<const RosterCandidate*> sorted;
    sorted.reserve(candidates.size());
    for (const RosterCandidate& candidate : candidates) {
        if (candidate.playerId == kInvalidPlayerId) continue;
        sorted.push_back(&candidate);
    }
    std::sort(sorted.begin(), sorted.end(), [](const RosterCandidate* a, const RosterCandidate* b) {
        return a->playerId < b->playerId;
    });

    Roster roster;
    for (std::size_t i = 0; i < sorted.size() && i < static_cast<std::size_t>(kMaxCars); ++i) {
        RosterEntry& entry = roster.entries[i];
        entry.playerId = sorted[i]->playerId;
        entry.seat = static_cast<std::uint8_t>(i);
        entry.name = sorted[i]->name.substr(0, kNameBytes - 1);
        ++roster.count;
    }
    return roster;
}

std::optional<std::uint8_t> SeatOf(const Roster& roster, PlayerId id) {
    if (id == kInvalidPlayerId) return std::nullopt;
    for (std::uint8_t i = 0; i < roster.count && i < static_cast<std::uint8_t>(kMaxCars); ++i) {
        if (roster.entries[i].playerId == id) return roster.entries[i].seat;
    }
    return std::nullopt;
}

}  // namespace apex::net
