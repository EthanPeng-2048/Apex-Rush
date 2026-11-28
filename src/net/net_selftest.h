#pragma once

namespace apex {

// `--net-selftest`: protocol + session checks that need no network stack and no
// window. Returns 0 when every check passed.
int RunNetSelfTest();

// `--net-disco-probe <seconds> [roomCode]`: windowless LAN discovery probe.
// Advertises a room when roomCode > 0, always searches, prints every room that
// turns up and the socket's own diagnostics. Returns 0 when a room was seen.
int RunDiscoveryProbe(int seconds, int roomCode);

}  // namespace apex
