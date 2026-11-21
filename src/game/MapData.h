#pragma once

#include <string>
#include <vector>

#include "core/Math.h"

namespace apex {

// Data-driven map: authored circuit data loaded from assets/maps/<name>/.
//  - track.csv    sparse centerline waypoints (engine smooths Catmull-Rom):
//                 x, z, y, width, grass_left, gravel_left, grass_right, gravel_right
//                 (runoff band widths from the road edge outward: grass, then
//                 gravel, then the wall. Left/right follow the generator's
//                 map convention: right = (dz, -dx) of the running direction.)
//  - terrain.csv  height posts x, z, y on a regular grid (bilinear lookup).
//  - decor.csv    type, x, z, y, scale (trees; y is a hint, the engine snaps
//                 decor to the terrain height).
struct MapData {
    struct WayPoint {
        Vec3 position;
        float width = 12.f;
        float grassLeft = 0.f, gravelLeft = 0.f;
        float grassRight = 0.f, gravelRight = 0.f;
    };

    struct Decor {
        int type = 0;  // 0 = tree
        Vec3 position;
        float scale = 1.f;
    };

    std::string name;
    std::vector<WayPoint> centerline;
    std::vector<Decor> decor;

    // Regular height-post grid (sorted unique axes + row-major heights).
    std::vector<float> gridX, gridZ;
    std::vector<float> gridY;  // gridZ.size() rows x gridX.size() cols

    bool ok = false;

    // Bilinear terrain height (gridY outside clamps to the edge value).
    float TerrainHeight(float x, float z) const;
    float MinTerrainY() const;
};

// Load mapsRoot/<name>/{track,terrain,decor}.csv. Missing optional files are
// tolerated (decor/terrain); track.csv is required for ok=true.
MapData LoadMapData(const std::string& mapsRoot, const std::string& name);

// Map folder names under mapsRoot (for the settings menu). Empty on failure.
std::vector<std::string> ListMaps(const std::string& mapsRoot);

}  // namespace apex
