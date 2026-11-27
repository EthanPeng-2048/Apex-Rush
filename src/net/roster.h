#pragma once

// Seat assignment for LAN races. Every machine (host included) derives the same
// seat layout from the same player list, so "who drives which car" never depends
// on arrival order, connection slot or on the host happening to be player 0.

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include "net/identity.h"
#include "net/protocol/wire.h"

namespace apex::net {

// One participant as the host knows it before the race: the session's arrival
// slot (dense-ish, gets holes when someone leaves) plus the id the client proved
// in Hello.
struct RosterCandidate {
    std::uint8_t slot = 0;
    PlayerId playerId = kInvalidPlayerId;
    std::string name;
};

// The frozen roster: seats 0..count-1, ordered by playerId ascending.
//
// Duplicate ids are not repaired here — the host refuses a second Hello carrying
// an id it already seated (RejectReason::DuplicatePeer), so `candidates` always
// holds distinct ids and the sort is a total order. A candidate with no id yet
// is dropped rather than given a seat.
Roster BuildRoster(std::span<const RosterCandidate> candidates);

// Seat held by `id`, or nothing when that player is not seated.
std::optional<std::uint8_t> SeatOf(const Roster& roster, PlayerId id);

}  // namespace apex::net
