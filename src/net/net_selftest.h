#pragma once

namespace apex {

// `--net-selftest`: protocol + session checks that need no network stack and no
// window. Returns 0 when every check passed.
int RunNetSelfTest();

}  // namespace apex
