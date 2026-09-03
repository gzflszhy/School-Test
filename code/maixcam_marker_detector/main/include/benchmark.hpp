#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace maixcam_marker {

struct BenchmarkStats {
    std::size_t samples = 0;
    std::size_t found_frames = 0;
    std::size_t dropped_frames = 0;
    double mean_processing_ms = 0.0;
    double p50_processing_ms = 0.0;
    double p95_processing_ms = 0.0;
    double p99_processing_ms = 0.0;
    double mean_latency_ms = 0.0;
    double p95_latency_ms = 0.0;
    double mean_effective_fps = 0.0;
    double one_percent_low_fps = 0.0;
};

class BenchmarkAccumulator {
public:
    explicit BenchmarkAccumulator(std::size_t capacity = 600);

    void add(double processing_ms, double latency_ms, double effective_fps,
             bool found, bool dropped = false);
    void reset();
    BenchmarkStats stats() const;
    std::string summary() const;
    std::size_t size() const noexcept { return samples_.size(); }

private:
    struct Sample {
        double processing_ms;
        double latency_ms;
        double fps;
        bool found;
        bool dropped;
    };
    std::size_t capacity_;
    std::deque<Sample> samples_;
};

}  // namespace maixcam_marker
