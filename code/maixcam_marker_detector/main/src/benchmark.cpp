#include "benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <vector>

namespace maixcam_marker {
namespace {

double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double index = p * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(index));
    const auto hi = static_cast<std::size_t>(std::ceil(index));
    const double fraction = index - static_cast<double>(lo);
    return values[lo] * (1.0 - fraction) + values[hi] * fraction;
}

double mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
}

}  // namespace

BenchmarkAccumulator::BenchmarkAccumulator(std::size_t capacity)
    : capacity_(std::max<std::size_t>(16, capacity)) {}

void BenchmarkAccumulator::add(double processing_ms, double latency_ms,
                               double effective_fps, bool found, bool dropped) {
    if (samples_.size() == capacity_) samples_.pop_front();
    samples_.push_back({std::max(0.0, processing_ms), std::max(0.0, latency_ms),
                        std::max(0.0, effective_fps), found, dropped});
}

void BenchmarkAccumulator::reset() { samples_.clear(); }

BenchmarkStats BenchmarkAccumulator::stats() const {
    BenchmarkStats result;
    result.samples = samples_.size();
    if (samples_.empty()) return result;

    std::vector<double> processing;
    std::vector<double> latency;
    std::vector<double> fps;
    processing.reserve(samples_.size());
    latency.reserve(samples_.size());
    fps.reserve(samples_.size());
    for (const auto& sample : samples_) {
        processing.push_back(sample.processing_ms);
        latency.push_back(sample.latency_ms);
        if (sample.fps > 0.0) fps.push_back(sample.fps);
        result.found_frames += sample.found ? 1U : 0U;
        result.dropped_frames += sample.dropped ? 1U : 0U;
    }
    result.mean_processing_ms = mean(processing);
    result.p50_processing_ms = percentile(processing, 0.50);
    result.p95_processing_ms = percentile(processing, 0.95);
    result.p99_processing_ms = percentile(processing, 0.99);
    result.mean_latency_ms = mean(latency);
    result.p95_latency_ms = percentile(latency, 0.95);
    result.mean_effective_fps = mean(fps);
    result.one_percent_low_fps = percentile(fps, 0.01);
    return result;
}

std::string BenchmarkAccumulator::summary() const {
    const BenchmarkStats s = stats();
    std::ostringstream out;
    out << std::fixed << std::setprecision(2)
        << "samples=" << s.samples << " found=" << s.found_frames
        << " dropped=" << s.dropped_frames
        << " processing_ms(mean/p50/p95/p99)=" << s.mean_processing_ms << '/'
        << s.p50_processing_ms << '/' << s.p95_processing_ms << '/'
        << s.p99_processing_ms << " latency_ms(mean/p95)=" << s.mean_latency_ms
        << '/' << s.p95_latency_ms << " fps(mean/1%low)="
        << s.mean_effective_fps << '/' << s.one_percent_low_fps;
    return out.str();
}

}  // namespace maixcam_marker
