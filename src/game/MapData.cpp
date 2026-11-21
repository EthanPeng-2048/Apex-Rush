#include "game/MapData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace apex {

namespace {

// Split a CSV line on commas into trimmed non-empty fields.
std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        // Trim spaces.
        const size_t b = field.find_first_not_of(" \t\r\n");
        const size_t e = field.find_last_not_of(" \t\r\n");
        out.push_back(b == std::string::npos ? "" : field.substr(b, e - b + 1));
    }
    return out;
}

bool IsCommentOrEmpty(const std::string& line) {
    for (char c : line) {
        if (c == '#') return true;
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') return false;
    }
    return true;
}

}  // namespace

float MapData::TerrainHeight(float x, float z) const {
    if (gridX.size() < 2 || gridZ.size() < 2 || gridY.empty()) return 0.f;
    auto locate = [](const std::vector<float>& axis, float v) {
        if (v <= axis.front()) return std::pair<size_t, float>{0, 0.f};
        if (v >= axis.back()) return std::pair<size_t, float>{axis.size() - 2, 1.f};
        const size_t i = static_cast<size_t>(
            std::upper_bound(axis.begin(), axis.end(), v) - axis.begin() - 1);
        const float t = (v - axis[i]) / std::max(1e-4f, axis[i + 1] - axis[i]);
        return std::pair<size_t, float>{i, t};
    };
    const auto [ix, tx] = locate(gridX, x);
    const auto [iz, tz] = locate(gridZ, z);
    const size_t cols = gridX.size();
    const float h00 = gridY[iz * cols + ix];
    const float h10 = gridY[iz * cols + ix + 1];
    const float h01 = gridY[(iz + 1) * cols + ix];
    const float h11 = gridY[(iz + 1) * cols + ix + 1];
    const float a = Lerp(h00, h10, tx);
    const float b = Lerp(h01, h11, tx);
    return Lerp(a, b, tz);
}

float MapData::MinTerrainY() const {
    float m = 0.f;
    for (float y : gridY) m = std::min(m, y);
    return m;
}

MapData LoadMapData(const std::string& mapsRoot, const std::string& name) {
    MapData map;
    map.name = name;
    const std::filesystem::path dir = std::filesystem::path(mapsRoot) / name;

    // track.csv — required.
    {
        std::ifstream in(dir / "track.csv");
        if (!in) return map;
        std::string line;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 8) continue;
            MapData::WayPoint w;
            w.position = {std::stof(f[0]), std::stof(f[2]), std::stof(f[1])};
            w.width = std::stof(f[3]);
            w.grassLeft = std::stof(f[4]);
            w.gravelLeft = std::stof(f[5]);
            w.grassRight = std::stof(f[6]);
            w.gravelRight = std::stof(f[7]);
            map.centerline.push_back(w);
        }
        if (map.centerline.size() < 3) return map;
    }

    // terrain.csv — optional height posts.
    {
        std::ifstream in(dir / "terrain.csv");
        std::string line;
        std::vector<float> xs, zs;
        std::vector<float> ys;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 3) continue;
            xs.push_back(std::stof(f[0]));
            zs.push_back(std::stof(f[1]));
            ys.push_back(std::stof(f[2]));
        }
        if (xs.size() >= 4) {
            map.gridX = xs;
            map.gridZ = zs;
            std::sort(map.gridX.begin(), map.gridX.end());
            map.gridX.erase(std::unique(map.gridX.begin(), map.gridX.end()), map.gridX.end());
            std::sort(map.gridZ.begin(), map.gridZ.end());
            map.gridZ.erase(std::unique(map.gridZ.begin(), map.gridZ.end()), map.gridZ.end());
            // Rebuild heights on the sorted unique grid (nearest raw post).
            const size_t cols = map.gridX.size();
            map.gridY.assign(cols * map.gridZ.size(), 0.f);
            for (size_t r = 0; r < map.gridZ.size(); ++r) {
                for (size_t c = 0; c < cols; ++c) {
                    // Find the raw row matching this (x,z) — the generator
                    // writes a regular grid so exact matches exist.
                    float best = 1e30f, bestY = 0.f;
                    for (size_t k = 0; k < xs.size(); ++k) {
                        const float d = std::fabs(xs[k] - map.gridX[c]) +
                                        std::fabs(zs[k] - map.gridZ[r]);
                        if (d < best) {
                            best = d;
                            bestY = ys[k];
                        }
                    }
                    map.gridY[r * cols + c] = bestY;
                }
            }
        }
    }

    // decor.csv — optional.
    {
        std::ifstream in(dir / "decor.csv");
        std::string line;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 5) continue;
            MapData::Decor d;
            d.type = (f[0] == "tree") ? 0 : 1;
            d.position = {std::stof(f[1]), std::stof(f[3]), std::stof(f[2])};
            d.scale = std::stof(f[4]);
            map.decor.push_back(d);
        }
    }

    map.ok = true;
    return map;
}

std::vector<std::string> ListMaps(const std::string& mapsRoot) {
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(mapsRoot, ec)) {
        if (!entry.is_directory()) continue;
        if (std::filesystem::exists(entry.path() / "track.csv")) {
            out.push_back(entry.path().filename().string());
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace apex
