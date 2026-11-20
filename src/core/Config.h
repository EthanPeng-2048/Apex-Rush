#pragma once

#include <string>
#include <unordered_map>

namespace apex {

// Minimal INI-style config persisted next to the EXE (config.ini).
class Config {
public:
    bool Load(const std::string& path);
    bool Save(const std::string& path) const;

    std::string GetString(const std::string& section, const std::string& key,
                          const std::string& fallback = "") const;
    float GetFloat(const std::string& section, const std::string& key, float fallback = 0.f) const;
    int GetInt(const std::string& section, const std::string& key, int fallback = 0) const;
    bool GetBool(const std::string& section, const std::string& key, bool fallback = false) const;

    void SetString(const std::string& section, const std::string& key, const std::string& value);

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> values_;
};

}  // namespace apex
