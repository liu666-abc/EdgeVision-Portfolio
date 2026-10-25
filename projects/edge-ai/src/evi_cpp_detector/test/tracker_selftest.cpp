#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "evi_cpp_detector/motion_tracker.hpp"

namespace {

void require(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "TRACKER_SELFTEST_FAIL " << message << std::endl;
        std::exit(1);
    }
}

evi_tracking::TrackDetection person(float x, float score = 0.8f) {
    evi_tracking::TrackDetection detection;
    detection.class_id = 0;
    detection.score = score;
    detection.x1 = x;
    detection.y1 = 100.0f;
    detection.x2 = x + 40.0f;
    detection.y2 = 200.0f;
    return detection;
}

}  // namespace

int main() {
    evi_tracking::MotionTracker tracker(0.20f, 60, 0.50f);
    std::vector<evi_tracking::TrackDetection> output;
    for (int i = 0; i < 5; ++i) {
        output = tracker.update({person(100.0f + static_cast<float>(i) * 4.0f)}, 1.0 / 30.0);
    }
    require(output.size() == 1, "expected one initial track");
    const int stable_id = output.front().track_id;
    require(stable_id > 0, "track id must be positive");

    for (int i = 0; i < 5; ++i) {
        output = tracker.update({}, 1.0 / 30.0);
    }
    output = tracker.update({person(124.0f)}, 1.0 / 30.0);
    require(output.size() == 1, "expected one recovered track");
    require(output.front().track_id == stable_id, "track id changed after misses");

    output = tracker.update({person(400.0f), person(124.0f)}, 1.0 / 30.0);
    output = tracker.update({person(400.0f), person(124.0f)}, 1.0 / 30.0);
    require(output.size() == 2, "expected two active tracks");
    int recovered_id = -1;
    int new_id = -1;
    for (const auto &item : output) {
        if (std::abs(item.x1 - 124.0f) < 12.0f) recovered_id = item.track_id;
        else new_id = item.track_id;
    }
    require(recovered_id == stable_id, "recovered track id mismatch");
    require(new_id > 0 && new_id != stable_id, "new track id invalid");
    std::cout << "TRACKER_SELFTEST_OK stable_id=" << stable_id
              << " new_id=" << new_id << std::endl;
    return 0;
}

