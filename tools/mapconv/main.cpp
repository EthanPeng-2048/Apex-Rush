// mapconv — convert a CSV map directory (from tools/make_test_track.py or
// hand-authored) into a runtime .rushmap (KVRec v2).
//
//   mapconv <in_csv_dir> <out.rushmap>
//
// Layering: tools -> rush::engine only (no direct kvrec include).

import rush.engine.core;

#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: mapconv <in_csv_dir> <out.rushmap>\n");
        return 2;
    }

    const std::string inDir = argv[1];
    const std::string outPath = argv[2];

    apex::MapData map = apex::LoadMapFromCsvDir(inDir);
    if (!map.ok) {
        std::fprintf(stderr, "mapconv: failed to load CSV map from '%s'\n", inDir.c_str());
        return 1;
    }

    if (!apex::SaveMapToFile(outPath, map)) {
        std::fprintf(stderr, "mapconv: failed to write '%s'\n", outPath.c_str());
        return 1;
    }

    std::printf("mapconv: %s -> %s (%zu waypoints, %zu decor, %zu drs zones)\n",
                inDir.c_str(), outPath.c_str(), map.centerline.size(), map.decor.size(),
                map.drsZones.size());
    return 0;
}
