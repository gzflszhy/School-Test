#pragma once

#include <array>
#include <chrono>
#include <vector>

#include <opencv2/core.hpp>

#include "benchmark.hpp"
#include "detection_result.hpp"
#include "detector_config.hpp"
#include "detector_state.hpp"
#include "marker_template.hpp"

namespace maixcam_marker {

struct DetectorDebugSnapshot {
    cv::Rect search_roi{};
    int led_threshold = 0;
    int raw_contour_count = 0;
    cv::Mat led_mask;
    std::vector<cv::RotatedRect> candidates;
};

class MarkerDetector {
public:
    explicit MarkerDetector(DetectorConfig config = DetectorConfig{});

    // capture_timestamp must describe this image, not the time process() is called.
    DetectionResult process(const cv::Mat& frame,
                            SteadyTimePoint capture_timestamp = std::chrono::steady_clock::now());
    void recordDroppedFrame();
    void reset();

    const DetectorConfig& config() const noexcept { return config_; }
    const DetectorStateData& stateData() const noexcept { return state_; }
    DetectorState state() const noexcept { return state_.mode; }
    const BenchmarkAccumulator& benchmark() const noexcept { return benchmark_; }
    void setDebugEnabled(bool enabled) noexcept { debug_enabled_ = enabled; }
    const DetectorDebugSnapshot& debugSnapshot() const noexcept { return debug_snapshot_; }

private:
    enum class HypothesisPath { LED_TRIPLE, DARK_BOARD };

    struct LedCandidate {
        std::vector<cv::Point> contour;
        cv::RotatedRect box;
        cv::Point2f center;
        float outer_size = 0.0F;
        float mean_intensity = 0.0F;
        float score = 0.0F;
    };

    struct Hypothesis {
        bool valid = false;
        float topology_score = 0.0F;
        float acceptance_score = 0.0F;
        float total_score = 0.0F;
        cv::Matx23f image_to_canonical{};
        cv::Matx23f canonical_to_image{};
        cv::RotatedRect rotated_board{};
        cv::Rect2f bbox{};
        cv::Point2f center{};
        MarkerOrientation orientation = MarkerOrientation::UNKNOWN;
        TemplateScore detail{};
        HypothesisPath path = HypothesisPath::LED_TRIPLE;
    };

    struct TripleProposal {
        std::array<cv::Point2f, 3> image_points{};
        float topology_score = 0.0F;
    };

    cv::Rect trackingSearchRoi(SteadyTimePoint capture_timestamp) const;
    void makeGray(const cv::Mat& frame);
    int makeLedMask(const cv::Rect& roi);
    std::vector<LedCandidate> findLedCandidates(const cv::Rect& roi);
    Hypothesis findBestLedHypothesis(const std::vector<LedCandidate>& candidates,
                                     const cv::Rect& roi);
    bool makeTripleProposal(const LedCandidate& a, const LedCandidate& b,
                            const LedCandidate& c, TripleProposal& proposal) const;
    Hypothesis evaluateTriple(const TripleProposal& proposal, int& transform_budget);
    Hypothesis findDarkBoardFallback(const cv::Rect& roi);
    Hypothesis evaluateTransform(const cv::Matx23f& image_to_canonical,
                                 float topology_score,
                                 MarkerOrientation orientation,
                                 HypothesisPath path);
    float temporalScore(const Hypothesis& hypothesis,
                        SteadyTimePoint capture_timestamp) const;
    DetectionResult makeResult(const Hypothesis& hypothesis, float confidence,
                               float acceptance_confidence,
                               SteadyTimePoint capture_timestamp);
    void updateState(const DetectionResult& result, SteadyTimePoint capture_timestamp);

    DetectorConfig config_;
    DetectorStateData state_;
    MarkerTemplate marker_template_;
    BenchmarkAccumulator benchmark_;

    cv::Mat gray_;
    cv::Mat roi_binary_;
    cv::Mat canonical_gray_;
    cv::Mat canonical_binary_;
    cv::Mat candidate_mask_;
    cv::Mat affine_inverse_;
    cv::Mat morphology_kernel_;
    std::vector<std::vector<cv::Point>> contour_buffer_;
    std::vector<cv::Point> hull_buffer_;
    std::vector<cv::Point> local_contour_buffer_;
    std::vector<cv::Point2f> board_corner_buffer_;
    std::vector<TripleProposal> triple_proposals_;
    bool debug_enabled_ = false;
    DetectorDebugSnapshot debug_snapshot_;
    SteadyTimePoint previous_output_{};
};

}  // namespace maixcam_marker
