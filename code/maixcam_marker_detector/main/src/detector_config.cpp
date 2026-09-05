#include "detector_config.hpp"

#include <cmath>
#include <fstream>
#include <string>

namespace maixcam_marker {
namespace {

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool parseInt(const std::string& text, int& value) {
    try {
        std::size_t consumed = 0;
        value = std::stoi(text, &consumed);
        return consumed == text.size();
    } catch (...) {
        return false;
    }
}

bool parseFloat(const std::string& text, float& value) {
    try {
        std::size_t consumed = 0;
        value = std::stof(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

bool parseBool(const std::string& text, bool& value) {
    int parsed = 0;
    if (!parseInt(text, parsed) || (parsed != 0 && parsed != 1)) return false;
    value = parsed != 0;
    return true;
}

}  // namespace

bool loadDetectorConfig(const std::string& path, DetectorConfig& config,
                        std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "cannot open detector config: " + path;
        return false;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        line = trim(line);
        if (line.empty()) continue;
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            error = "detector config line " + std::to_string(line_number) +
                    " has no '='";
            return false;
        }
        const std::string key = trim(line.substr(0, equals));
        const std::string text = trim(line.substr(equals + 1));

#define SET_INT(name)                                      \
        if (key == #name) {                               \
            if (!parseInt(text, config.name)) {           \
                error = "invalid integer for '" #name    \
                        "' on line " + std::to_string(line_number); \
                return false;                             \
            }                                             \
            continue;                                     \
        }
#define SET_FLOAT(name)                                    \
        if (key == #name) {                               \
            if (!parseFloat(text, config.name)) {         \
                error = "invalid number for '" #name     \
                        "' on line " + std::to_string(line_number); \
                return false;                             \
            }                                             \
            continue;                                     \
        }
#define SET_BOOL(name)                                     \
        if (key == #name) {                               \
            if (!parseBool(text, config.name)) {          \
                error = "'" #name "' must be 0 or 1 on line " + \
                        std::to_string(line_number);       \
                return false;                             \
            }                                             \
            continue;                                     \
        }
        SET_BOOL(use_auto_exposure)
        SET_INT(exposure_us)
        SET_BOOL(use_fixed_led_threshold)
        SET_INT(fixed_led_threshold)
        SET_FLOAT(bright_percentile)
        SET_INT(min_led_threshold)
        SET_INT(max_led_threshold)
        SET_INT(local_contrast_threshold)
        SET_INT(saturation_threshold)
        SET_INT(morphology_kernel)
#undef SET_BOOL
#undef SET_FLOAT
#undef SET_INT
        error = "unknown detector config key '" + key + "' on line " +
                std::to_string(line_number);
        return false;
    }
    config.normalize();
    if (!config.use_auto_exposure && config.exposure_us <= 0) {
        error = "manual exposure_us must be positive";
        return false;
    }
    return true;
}

}  // namespace maixcam_marker
