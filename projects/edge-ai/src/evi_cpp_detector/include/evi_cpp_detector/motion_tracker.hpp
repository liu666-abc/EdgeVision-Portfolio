#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/opencv.hpp>

namespace evi_tracking {

struct TrackDetection {
    int class_id = -1;
    float score = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    int track_id = -1;
};

inline float iou(const TrackDetection &a, const TrackDetection &b) {
    const float x1 = std::max(a.x1, b.x1);
    const float y1 = std::max(a.y1, b.y1);
    const float x2 = std::min(a.x2, b.x2);
    const float y2 = std::min(a.y2, b.y2);
    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float area_a =
        std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    const float area_b =
        std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    const float united = area_a + area_b - inter;
    return united > 0.0f ? inter / united : 0.0f;
}

class MotionTracker {
public:
    explicit MotionTracker(
        float iou_threshold = 0.20f,
        int max_misses = 60,
        float high_score = 0.50f)
        : iou_threshold_(iou_threshold),
          max_misses_(max_misses),
          high_score_(high_score) {}

    std::vector<TrackDetection> update(
        const std::vector<TrackDetection> &detections,
        double dt_s) {
        const double dt = std::clamp(dt_s, 1.0 / 120.0, 0.5);
        for (Track &track : tracks_) {
            track.predicted = predict(track, dt);
        }

        std::vector<int> track_match(tracks_.size(), -1);
        std::vector<int> detection_match(detections.size(), -1);

        std::vector<int> high;
        std::vector<int> low;
        for (size_t i = 0; i < detections.size(); ++i) {
            if (detections[i].score >= high_score_) high.push_back(static_cast<int>(i));
            else low.push_back(static_cast<int>(i));
        }
        associate(detections, high, track_match, detection_match);

        std::vector<int> unmatched_tracks;
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (track_match[i] < 0) unmatched_tracks.push_back(static_cast<int>(i));
        }
        associate(
            detections,
            low,
            track_match,
            detection_match,
            &unmatched_tracks);

        for (size_t i = 0; i < tracks_.size(); ++i) {
            Track &track = tracks_[i];
            if (track_match[i] >= 0) {
                TrackDetection updated = detections[track_match[i]];
                update(track, updated);
            } else {
                track.misses += 1;
            }
        }

        for (size_t i = 0; i < detections.size(); ++i) {
            if (detections[i].score < high_score_ || detection_match[i] >= 0) {
                continue;
            }
            tracks_.push_back(make_track(detections[i]));
        }

        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),
                [this](const Track &track) {
                    return track.misses > max_misses_;
                }),
            tracks_.end());

        std::vector<TrackDetection> output;
        output.reserve(tracks_.size());
        for (const Track &track : tracks_) {
            if (!track.confirmed) continue;
            TrackDetection item = track.predicted;
            item.class_id = track.class_id;
            item.score = track.score;
            item.track_id = track.id;
            output.push_back(item);
        }
        std::sort(
            output.begin(),
            output.end(),
            [](const TrackDetection &a, const TrackDetection &b) {
                return a.score > b.score;
            });
        return output;
    }

    size_t track_count() const { return tracks_.size(); }

private:
    struct Track {
        int id = -1;
        int class_id = -1;
        float score = 0.0f;
        int misses = 0;
        int hits = 0;
        bool confirmed = false;
        cv::KalmanFilter filter{8, 4, 0, CV_32F};
        TrackDetection predicted;
    };

    cv::Mat state_from_box(const TrackDetection &detection) const {
        cv::Mat state(8, 1, CV_32F, cv::Scalar(0));
        state.at<float>(0) = (detection.x1 + detection.x2) * 0.5f;
        state.at<float>(1) = (detection.y1 + detection.y2) * 0.5f;
        state.at<float>(2) = std::max(1.0f, detection.x2 - detection.x1);
        state.at<float>(3) = std::max(1.0f, detection.y2 - detection.y1);
        return state;
    }

    TrackDetection box_from_state(const cv::Mat &state, int class_id) const {
        const float cx = state.at<float>(0);
        const float cy = state.at<float>(1);
        const float width = std::max(1.0f, state.at<float>(2));
        const float height = std::max(1.0f, state.at<float>(3));
        TrackDetection result;
        result.class_id = class_id;
        result.x1 = cx - width * 0.5f;
        result.y1 = cy - height * 0.5f;
        result.x2 = cx + width * 0.5f;
        result.y2 = cy + height * 0.5f;
        return result;
    }

    Track make_track(const TrackDetection &detection) {
        Track track;
        track.id = next_id_++;
        track.class_id = detection.class_id;
        track.score = detection.score;
        track.hits = 1;
        track.predicted = detection;
        cv::setIdentity(track.filter.transitionMatrix, cv::Scalar(1));
        cv::setIdentity(track.filter.measurementMatrix, cv::Scalar(1));
        track.filter.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
        for (int i = 0; i < 4; ++i) {
            track.filter.transitionMatrix.at<float>(i, i + 4) = 1.0f / 30.0f;
        }
        track.filter.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
        for (int i = 0; i < 4; ++i) {
            track.filter.measurementMatrix.at<float>(i, i) = 1.0f;
        }
        cv::setIdentity(track.filter.processNoiseCov, cv::Scalar(1e-2));
        cv::setIdentity(track.filter.measurementNoiseCov, cv::Scalar(1e-1));
        cv::setIdentity(track.filter.errorCovPost, cv::Scalar(1));
        track.filter.statePost = state_from_box(detection);
        track.predicted.track_id = track.id;
        return track;
    }

    TrackDetection predict(Track &track, double dt) {
        track.filter.transitionMatrix =
            cv::Mat::eye(8, 8, CV_32F);
        for (int i = 0; i < 4; ++i) {
            track.filter.transitionMatrix.at<float>(i, i + 4) =
                static_cast<float>(dt);
        }
        const cv::Mat state = track.filter.predict();
        TrackDetection predicted = box_from_state(state, track.class_id);
        predicted.track_id = track.id;
        predicted.score = track.score;
        return predicted;
    }

    void update(Track &track, const TrackDetection &detection) {
        cv::Mat measurement(4, 1, CV_32F);
        measurement.at<float>(0) = (detection.x1 + detection.x2) * 0.5f;
        measurement.at<float>(1) = (detection.y1 + detection.y2) * 0.5f;
        measurement.at<float>(2) = std::max(1.0f, detection.x2 - detection.x1);
        measurement.at<float>(3) = std::max(1.0f, detection.y2 - detection.y1);
        track.filter.correct(measurement);
        track.score = detection.score;
        track.misses = 0;
        track.hits += 1;
        if (track.hits >= 2) track.confirmed = true;
        track.predicted = box_from_state(track.filter.statePost, track.class_id);
        track.predicted.track_id = track.id;
        track.predicted.score = track.score;
    }

    void associate(
        const std::vector<TrackDetection> &detections,
        const std::vector<int> &detection_indices,
        std::vector<int> &track_match,
        std::vector<int> &detection_match,
        const std::vector<int> *track_subset = nullptr) {
        struct Candidate {
            float overlap;
            int track;
            int detection;
        };
        std::vector<Candidate> candidates;
        for (size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
            if (track_match[track_index] >= 0) continue;
            if (
                track_subset &&
                std::find(
                    track_subset->begin(),
                    track_subset->end(),
                    static_cast<int>(track_index)) == track_subset->end()) {
                continue;
            }
            for (int detection_index : detection_indices) {
                if (detection_match[detection_index] >= 0) continue;
                if (
                    tracks_[track_index].class_id !=
                    detections[detection_index].class_id) {
                    continue;
                }
                const TrackDetection &track_box = tracks_[track_index].predicted;
                const TrackDetection &detection_box = detections[detection_index];
                const float overlap = iou(track_box, detection_box);
                const float track_cx = (track_box.x1 + track_box.x2) * 0.5f;
                const float track_cy = (track_box.y1 + track_box.y2) * 0.5f;
                const float detection_cx =
                    (detection_box.x1 + detection_box.x2) * 0.5f;
                const float detection_cy =
                    (detection_box.y1 + detection_box.y2) * 0.5f;
                const float average_width = std::max(
                    8.0f,
                    ((track_box.x2 - track_box.x1) +
                     (detection_box.x2 - detection_box.x1)) * 0.5f);
                const float average_height = std::max(
                    8.0f,
                    ((track_box.y2 - track_box.y1) +
                     (detection_box.y2 - detection_box.y1)) * 0.5f);
                const float normalized_distance = std::sqrt(
                    std::pow((track_cx - detection_cx) / average_width, 2.0f) +
                    std::pow((track_cy - detection_cy) / average_height, 2.0f));
                const bool geometry_match =
                    overlap >= iou_threshold_ || normalized_distance <= 0.85f;
                if (geometry_match) {
                    candidates.push_back(
                        {
                            overlap +
                                0.30f * std::max(0.0f, 1.5f - normalized_distance),
                            static_cast<int>(track_index),
                            detection_index,
                        });
                }
            }
        }
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const Candidate &a, const Candidate &b) {
                return a.overlap > b.overlap;
            });
        for (const Candidate &candidate : candidates) {
            if (
                track_match[candidate.track] >= 0 ||
                detection_match[candidate.detection] >= 0) {
                continue;
            }
            track_match[candidate.track] = candidate.detection;
            detection_match[candidate.detection] = candidate.track;
        }
    }

    float iou_threshold_;
    int max_misses_;
    float high_score_;
    int next_id_ = 1;
    std::vector<Track> tracks_;
};

}  // namespace evi_tracking

