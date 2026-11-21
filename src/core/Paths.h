#pragma once

#include <string>

namespace apex {

// Data root for config.ini and assets/: resolved once at first use so the
// game finds its data regardless of the working directory. Search order:
//   1. current directory (normal in-tree runs)
//   2. the executable's directory (shipped layout: exe next to assets/)
//   3. the executable's parent (running build\apex-rush.exe in-tree)
// The first root containing assets/maps wins; config.ini lives in the same
// root so settings never split across directories.
const std::string& DataRoot();

// DataRoot() + "/" + rel ("config.ini", "assets/maps", ...).
std::string DataPath(const std::string& rel);

}  // namespace apex
