// ROS2 C++ node: static image -> preprocess -> BPU -> DFL decode -> NMS -> publish.
// Static-image input only; no chassis, no /cmd_vel, no camera.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

#include "evi_cpp_detector/msg/detection_array.hpp"
#include "evi_cpp_detector/motion_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace {

constexpr int kImageSize = 640;
constexpr int kDflBins = 16;
constexpr int kNumClasses = 80;

using RawDetection = evi_tracking::TrackDetection;

float sigmoid(float value) { return 1.0f / (1.0f + std::exp(-value)); }

double elapsed_ms(const std::chrono::steady_clock::time_point &from) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - from).count();
}

void bgr_to_nv12(const cv::Mat &bgr, std::vector<uint8_t> &nv12) {
    const int width = bgr.cols;
    const int height = bgr.rows;
    nv12.assign(static_cast<size_t>(width) * height * 3 / 2, 0);
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = nv12.data() + static_cast<size_t>(width) * height;
    for (int row = 0; row < height; ++row) {
        const cv::Vec3b *src = bgr.ptr<cv::Vec3b>(row);
        for (int col = 0; col < width; ++col) {
            const float b = src[col][0], g = src[col][1], r = src[col][2];
            y_plane[row * width + col] = static_cast<uint8_t>(std::min(
                255.0f, std::max(0.0f, 0.299f * r + 0.587f * g + 0.114f * b)));
        }
    }
    for (int row = 0; row + 1 < height; row += 2) {
        for (int col = 0; col + 1 < width; col += 2) {
            float r = 0, g = 0, b = 0;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    const cv::Vec3b px = bgr.at<cv::Vec3b>(row + dy, col + dx);
                    b += px[0]; g += px[1]; r += px[2];
                }
            }
            r /= 4; g /= 4; b /= 4;
            const int u = static_cast<int>(std::min(255.0f, std::max(
                0.0f, -0.1687f * r - 0.3313f * g + 0.5f * b + 128.0f)));
            const int v = static_cast<int>(std::min(255.0f, std::max(
                0.0f, 0.5f * r - 0.4187f * g - 0.0813f * b + 128.0f)));
            const int base = (row / 2) * width + col;
            uv_plane[base] = static_cast<uint8_t>(u);
            uv_plane[base + 1] = static_cast<uint8_t>(v);
        }
    }
}

// Full-range BT.601 in 16-bit fixed point; same convention as bgr_to_nv12,
// but with pointer walks so the compiler can vectorise.
void bgr_to_nv12_fast(const cv::Mat &bgr, std::vector<uint8_t> &nv12) {
    const int width = bgr.cols;
    const int height = bgr.rows;
    nv12.resize(static_cast<size_t>(width) * height * 3 / 2);
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = nv12.data() + static_cast<size_t>(width) * height;
    for (int row = 0; row < height; ++row) {
        const uint8_t *src = bgr.ptr<uint8_t>(row);
        uint8_t *dst = y_plane + static_cast<size_t>(row) * width;
        for (int col = 0; col < width; ++col, src += 3) {
            dst[col] = static_cast<uint8_t>(
                (77 * src[2] + 150 * src[1] + 29 * src[0] + 128) >> 8);
        }
    }
    for (int row = 0; row + 1 < height; row += 2) {
        const uint8_t *r0 = bgr.ptr<uint8_t>(row);
        const uint8_t *r1 = bgr.ptr<uint8_t>(row + 1);
        uint8_t *dst = uv_plane + static_cast<size_t>(row / 2) * width;
        for (int col = 0; col + 1 < width; col += 2, r0 += 6, r1 += 6) {
            const int b = r0[0] + r0[3] + r1[0] + r1[3];
            const int g = r0[1] + r0[4] + r1[1] + r1[4];
            const int r = r0[2] + r0[5] + r1[2] + r1[5];
            int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 10);
            int v = 128 + ((128 * r - 107 * g - 21 * b) >> 10);
            dst[col] = static_cast<uint8_t>(std::min(255, std::max(0, u)));
            dst[col + 1] = static_cast<uint8_t>(std::min(255, std::max(0, v)));
        }
    }
}

// OpenCV SIMD path: BGR -> I420 (BT.601 video range) then interleave chroma.
void bgr_to_nv12_cv(const cv::Mat &bgr, std::vector<uint8_t> &nv12, cv::Mat &scratch) {
    const int width = bgr.cols;
    const int height = bgr.rows;
    cv::cvtColor(bgr, scratch, cv::COLOR_BGR2YUV_I420);
    nv12.resize(static_cast<size_t>(width) * height * 3 / 2);
    std::memcpy(nv12.data(), scratch.data, static_cast<size_t>(width) * height);
    const uint8_t *u_plane = scratch.data + static_cast<size_t>(width) * height;
    const uint8_t *v_plane = u_plane + static_cast<size_t>(width / 2) * (height / 2);
    uint8_t *uv_plane = nv12.data() + static_cast<size_t>(width) * height;
    const int pairs = (width / 2) * (height / 2);
    for (int i = 0; i < pairs; ++i) {
        uv_plane[2 * i] = u_plane[i];
        uv_plane[2 * i + 1] = v_plane[i];
    }
}

void letterbox(const cv::Mat &src, cv::Mat &dst, float &scale, int &pad_x, int &pad_y) {
    scale = std::min(kImageSize / static_cast<float>(src.cols),
                     kImageSize / static_cast<float>(src.rows));
    const int new_w = static_cast<int>(std::round(src.cols * scale));
    const int new_h = static_cast<int>(std::round(src.rows * scale));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    dst = cv::Mat::zeros(kImageSize, kImageSize, src.type());
    dst.setTo(cv::Scalar(114, 114, 114));
    pad_x = (kImageSize - new_w) / 2;
    pad_y = (kImageSize - new_h) / 2;
    resized.copyTo(dst(cv::Rect(pad_x, pad_y, new_w, new_h)));
}

float box_iou(const RawDetection &a, const RawDetection &b) {
    const float x1 = std::max(a.x1, b.x1);
    const float y1 = std::max(a.y1, b.y1);
    const float x2 = std::min(a.x2, b.x2);
    const float y2 = std::min(a.y2, b.y2);
    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    const float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

std::vector<RawDetection> nms(std::vector<RawDetection> input, float iou_threshold) {
    std::sort(input.begin(), input.end(),
              [](const RawDetection &a, const RawDetection &b) { return a.score > b.score; });
    std::vector<RawDetection> keep;
    std::vector<bool> removed(input.size(), false);
    for (size_t i = 0; i < input.size(); ++i) {
        if (removed[i]) continue;
        keep.push_back(input[i]);
        for (size_t j = i + 1; j < input.size(); ++j) {
            if (removed[j] || input[j].class_id != input[i].class_id) continue;
            if (box_iou(input[i], input[j]) > iou_threshold) removed[j] = true;
        }
    }
    return keep;
}

struct FrameResult {
    std::vector<RawDetection> detections;
    float read_ms = 0.0f;
    float letterbox_ms = 0.0f;
    float nv12_ms = 0.0f;
    float preprocess_ms = 0.0f;
    float infer_ms = 0.0f;
    float decode_ms = 0.0f;
    float total_ms = 0.0f;
};

}  // namespace

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("cpp_detector_node") {
        model_path_ = declare_parameter<std::string>("model", "");
        image_dir_ = declare_parameter<std::string>("image_dir", "");
        confidence_ = declare_parameter<double>("confidence", 0.25);
        iou_ = declare_parameter<double>("iou", 0.45);
        const double clamped = std::min(0.999, std::max(1e-4, confidence_));
        logit_threshold_ = static_cast<float>(std::log(clamped / (1.0 - clamped)));
        top_k_ = declare_parameter<int>("top_k", 300);
        const double period_s = declare_parameter<double>("period_s", 0.2);
        const bool loop = declare_parameter<bool>("loop", true);
        loop_ = loop;
        mode_ = declare_parameter<std::string>("mode", "static");
        image_topic_ = declare_parameter<std::string>(
            "image_topic", "/camera/color/image_raw");
        tracking_enabled_ = declare_parameter<bool>(
            "tracking", mode_ == "camera");
        const int image_qos_depth = declare_parameter<int>(
            "image_qos_depth", 1);
        const bool image_qos_reliable = declare_parameter<bool>(
            "image_qos_reliable", false);
        csv_path_ = declare_parameter<std::string>("csv_path", "");
        dump_json_dir_ = declare_parameter<std::string>("dump_json_dir", "");
        track_log_path_ = declare_parameter<std::string>("track_log_path", "");
        // "scalar" is the reference implementation validated against COCO GT; the
        // faster variants change the pixel values and are opt-in only (see README).
        nv12_mode_ = declare_parameter<std::string>("nv12_mode", "scalar");
        if (!csv_path_.empty()) {
            const bool needs_header = !std::filesystem::exists(csv_path_);
            csv_.open(csv_path_, std::ios::app);
            if (needs_header) {
                csv_ << "seq,image,detections,read_ms,letterbox_ms,nv12_ms,"
                        "preprocess_ms,infer_ms,decode_ms,total_ms\n";
            }
        }
        if (!track_log_path_.empty()) {
            track_log_.open(track_log_path_, std::ios::out | std::ios::trunc);
            if (!track_log_.is_open()) {
                throw std::runtime_error("cannot open track_log_path");
            }
        }

        if (mode_ == "static") {
            for (const auto &entry : std::filesystem::directory_iterator(image_dir_)) {
                const std::string path = entry.path().string();
                if (path.size() > 4 && (path.compare(path.size() - 4, 4, ".jpg") == 0 ||
                                        path.compare(path.size() - 4, 4, ".png") == 0)) {
                    images_.push_back(path);
                }
            }
            std::sort(images_.begin(), images_.end());
            if (images_.empty()) {
                RCLCPP_ERROR(get_logger(), "no images found in %s", image_dir_.c_str());
                throw std::runtime_error("no images");
            }
        } else if (mode_ != "camera") {
            throw std::runtime_error("mode must be static or camera");
        }

        load_model();
        publisher_ = create_publisher<evi_cpp_detector::msg::DetectionArray>(
            "/evi/detections", 10);
        if (mode_ == "camera") {
            auto qos = rclcpp::QoS(
                rclcpp::KeepLast(std::max(1, image_qos_depth)));
            if (image_qos_reliable) {
                qos.reliable();
            } else {
                qos.best_effort();
            }
            image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, qos,
                [this](sensor_msgs::msg::Image::SharedPtr message) {
                    on_image(message);
                });
            RCLCPP_INFO(get_logger(),
                        "cpp_detector_node up: camera=%s conf=%.2f iou=%.2f",
                        image_topic_.c_str(), confidence_, iou_);
            RCLCPP_INFO(
                get_logger(),
                "image_qos depth=%d reliable=%s",
                image_qos_depth,
                image_qos_reliable ? "true" : "false");
        } else {
            timer_ = create_wall_timer(
                std::chrono::duration<double>(period_s), [this]() { tick(); });
            RCLCPP_INFO(get_logger(),
                        "cpp_detector_node up: %zu images, conf=%.2f iou=%.2f",
                        images_.size(), confidence_, iou_);
        }
        RCLCPP_INFO(get_logger(), "nv12_mode=%s", nv12_mode_.c_str());
    }

private:
    void load_model() {
        const char *files[1] = {model_path_.c_str()};
        if (hbDNNInitializeFromFiles(&packed_, files, 1) != 0) {
            throw std::runtime_error("model load failed");
        }
        const char **names = nullptr;
        int model_count = 0;
        hbDNNGetModelNameList(&names, &model_count, packed_);
        hbDNNGetModelHandle(&dnn_, packed_, names[0]);
        hbDNNGetInputTensorProperties(&in_props_, dnn_, 0);
        in_bytes_ = 3 * in_props_.validShape.dimensionSize[2] *
                    in_props_.validShape.dimensionSize[3] / 2;
        hbDNNGetOutputCount(&out_count_, dnn_);
        outs_.resize(out_count_);
        for (int i = 0; i < out_count_; ++i) {
            hbDNNGetOutputTensorProperties(&outs_[i].properties, dnn_, i);
            hbSysAllocCachedMem(&outs_[i].sysMem[0], outs_[i].properties.alignedByteSize);
        }
        input_.properties = in_props_;
        hbSysAllocCachedMem(&input_.sysMem[0], in_bytes_);
        HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control_);
    }

    FrameResult process_image(const std::string &image_path) {
        FrameResult result;
        auto frame_start = std::chrono::steady_clock::now();

        auto stage = std::chrono::steady_clock::now();
        cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
        if (bgr.empty()) return result;
        result.read_ms = static_cast<float>(elapsed_ms(stage));
        result = process_frame(bgr, result.read_ms, frame_start);
        return result;
    }

    FrameResult process_frame(
        const cv::Mat &bgr,
        float read_ms,
        std::chrono::steady_clock::time_point frame_start) {
        FrameResult result;
        result.read_ms = read_ms;

        auto stage = std::chrono::steady_clock::now();
        cv::Mat padded;
        float scale = 1.0f;
        int pad_x = 0, pad_y = 0;
        letterbox(bgr, padded, scale, pad_x, pad_y);
        result.letterbox_ms = static_cast<float>(elapsed_ms(stage));

        stage = std::chrono::steady_clock::now();
        std::vector<uint8_t> nv12;
        if (nv12_mode_ == "scalar") {
            bgr_to_nv12(padded, nv12);
        } else if (nv12_mode_ == "fast") {
            bgr_to_nv12_fast(padded, nv12);
        } else {
            bgr_to_nv12_cv(padded, nv12, nv12_scratch_);
        }
        std::memcpy(input_.sysMem[0].virAddr, nv12.data(), in_bytes_);
        hbSysFlushMem(&input_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        result.nv12_ms = static_cast<float>(elapsed_ms(stage));
        result.preprocess_ms = result.read_ms + result.letterbox_ms + result.nv12_ms;

        stage = std::chrono::steady_clock::now();
        hbDNNTensor *out_ptr = outs_.data();
        hbDNNTaskHandle_t task = nullptr;
        if (hbDNNInfer(&task, &out_ptr, &input_, dnn_, &control_) != 0) return result;
        hbDNNWaitTaskDone(task, 0);
        hbDNNReleaseTask(task);
        result.infer_ms = static_cast<float>(elapsed_ms(stage));

        stage = std::chrono::steady_clock::now();
        std::vector<RawDetection> candidates;
        for (int pair = 0; pair < 3; ++pair) {
            const int cls_index = pair * 2;
            const int reg_index = pair * 2 + 1;
            hbSysFlushMem(&outs_[cls_index].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
            hbSysFlushMem(&outs_[reg_index].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
            const hbDNNTensorShape cls_shape = outs_[cls_index].properties.validShape;
            const int cells_h = cls_shape.dimensionSize[1];
            const int cells_w = cls_shape.dimensionSize[2];
            const float stride = kImageSize / static_cast<float>(cells_h);
            const float *cls_ptr =
                static_cast<const float *>(outs_[cls_index].sysMem[0].virAddr);
            const int32_t *reg_ptr =
                static_cast<const int32_t *>(outs_[reg_index].sysMem[0].virAddr);
            const float *scales = outs_[reg_index].properties.scale.scaleData;
            const int scale_len = outs_[reg_index].properties.scale.scaleLen;
            for (int row = 0; row < cells_h; ++row) {
                for (int col = 0; col < cells_w; ++col) {
                    // Sigmoid is monotonic, so argmax and thresholding can be done on the
                    // raw logits; only the surviving cell pays for one exp().
                    const int cls_base = (row * cells_w + col) * kNumClasses;
                    int best_class = -1;
                    float best_logit = -1e30f;
                    for (int c = 0; c < kNumClasses; ++c) {
                        const float value = cls_ptr[cls_base + c];
                        if (value > best_logit) {
                            best_logit = value;
                            best_class = c;
                        }
                    }
                    if (best_logit < logit_threshold_) continue;
                    const float best_score = sigmoid(best_logit);
                    const int reg_base = (row * cells_w + col) * (4 * kDflBins);
                    float side[4];
                    for (int s = 0; s < 4; ++s) {
                        float prob[kDflBins];
                        float max_value = -1e30f;
                        for (int b = 0; b < kDflBins; ++b) {
                            const int channel = s * kDflBins + b;
                            const float s_scale =
                                (channel < scale_len) ? scales[channel] : 1.0f;
                            prob[b] = static_cast<float>(reg_ptr[reg_base + channel]) * s_scale;
                            max_value = std::max(max_value, prob[b]);
                        }
                        float sum = 0.0f;
                        for (int b = 0; b < kDflBins; ++b) {
                            prob[b] = std::exp(prob[b] - max_value);
                            sum += prob[b];
                        }
                        float expectation = 0.0f;
                        for (int b = 0; b < kDflBins; ++b) {
                            expectation += (prob[b] / sum) * static_cast<float>(b);
                        }
                        side[s] = expectation;
                    }
                    const float anchor_x = (col + 0.5f) * stride;
                    const float anchor_y = (row + 0.5f) * stride;
                    RawDetection det;
                    det.class_id = best_class;
                    det.score = best_score;
                    det.x1 = (anchor_x - side[0] * stride - pad_x) / scale;
                    det.y1 = (anchor_y - side[1] * stride - pad_y) / scale;
                    det.x2 = (anchor_x + side[2] * stride - pad_x) / scale;
                    det.y2 = (anchor_y + side[3] * stride - pad_y) / scale;
                    candidates.push_back(det);
                }
            }
        }
        std::vector<RawDetection> kept = nms(candidates, iou_);
        std::sort(kept.begin(), kept.end(),
                  [](const RawDetection &a, const RawDetection &b) { return a.score > b.score; });
        if (static_cast<int>(kept.size()) > top_k_) kept.resize(top_k_);
        result.detections = kept;
        result.decode_ms = static_cast<float>(elapsed_ms(stage));
        result.total_ms = static_cast<float>(elapsed_ms(frame_start));
        return result;
    }

    bool image_to_bgr(
        const sensor_msgs::msg::Image::SharedPtr &message,
        cv::Mat &bgr) {
        const int type = CV_MAKETYPE(
            CV_8U,
            static_cast<int>(message->step / std::max<uint32_t>(message->width, 1)));
        if (message->encoding == "bgr8") {
            cv::Mat view(
                static_cast<int>(message->height),
                static_cast<int>(message->width),
                type,
                const_cast<uint8_t *>(message->data.data()),
                message->step);
            bgr = view.clone();
            return true;
        }
        if (message->encoding == "rgb8") {
            cv::Mat view(
                static_cast<int>(message->height),
                static_cast<int>(message->width),
                type,
                const_cast<uint8_t *>(message->data.data()),
                message->step);
            cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
            return true;
        }
        if (message->encoding == "mono8") {
            cv::Mat view(
                static_cast<int>(message->height),
                static_cast<int>(message->width),
                CV_8UC1,
                const_cast<uint8_t *>(message->data.data()),
                message->step);
            cv::cvtColor(view, bgr, cv::COLOR_GRAY2BGR);
            return true;
        }
        return false;
    }

    void on_image(const sensor_msgs::msg::Image::SharedPtr &message) {
        auto started = std::chrono::steady_clock::now();
        cv::Mat bgr;
        if (!image_to_bgr(message, bgr)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "unsupported image encoding: %s", message->encoding.c_str());
            return;
        }
        const float read_ms = static_cast<float>(elapsed_ms(started));
        const FrameResult result = process_frame(bgr, read_ms, started);
        publish_result(result, std::string("camera"), rclcpp::Time(message->header.stamp));
        if (csv_.is_open()) {
            csv_ << published_ << ",camera," << result.detections.size()
                 << "," << result.read_ms << "," << result.letterbox_ms << ","
                 << result.nv12_ms << "," << result.preprocess_ms << ","
                 << result.infer_ms << "," << result.decode_ms << ","
                 << result.total_ms << "\n";
            csv_.flush();
        }
        if (!dump_json_dir_.empty()) {
            dump_frame_json(published_, "camera", result);
        }
    }

    void dump_frame_json(size_t seq, const std::string &image_path,
                         const FrameResult &result) {
        std::filesystem::create_directories(dump_json_dir_);
        const std::string stem = std::filesystem::path(image_path).stem().string();
        const std::string path =
            dump_json_dir_ + "/" + std::to_string(seq) + "_" + stem + ".json";
        std::ofstream out(path);
        out << "{\n";
        out << "  \"image_path\": \"" << image_path << "\",\n";
        out << "  \"confidence_threshold\": " << confidence_ << ",\n";
        out << "  \"iou_threshold\": " << iou_ << ",\n";
        out << "  \"detection_count\": " << result.detections.size() << ",\n";
        out << "  \"detections\": [\n";
        for (size_t i = 0; i < result.detections.size(); ++i) {
            const RawDetection &det = result.detections[i];
            out << "    {\"class_id\": " << det.class_id
                << ", \"score\": " << std::setprecision(6) << det.score
                << ", \"bbox_xyxy\": [" << det.x1 << ", " << det.y1 << ", "
                << det.x2 << ", " << det.y2 << "]}";
            if (i + 1 < result.detections.size()) out << ",";
            out << "\n";
        }
        out << "  ]\n}\n";
        out.close();
    }

    void publish_result(
        FrameResult result,
        const std::string &image_path,
        const rclcpp::Time &stamp) {
        if (tracking_enabled_) {
            const auto now_steady = std::chrono::steady_clock::now();
            const double dt =
                last_tracking_time_.time_since_epoch().count() == 0
                    ? 1.0 / 30.0
                    : std::chrono::duration<double>(
                          now_steady - last_tracking_time_).count();
            last_tracking_time_ = now_steady;
            result.detections = tracker_.update(result.detections, dt);
        }
        evi_cpp_detector::msg::DetectionArray message;
        message.header.stamp = stamp;
        message.header.frame_id = "cpp_detector";
        message.image_path = image_path;
        message.preprocess_ms = result.preprocess_ms;
        message.infer_ms = result.infer_ms;
        message.decode_ms = result.decode_ms;
        message.total_ms = result.total_ms;
        message.detections.reserve(result.detections.size());
        for (const RawDetection &det : result.detections) {
            evi_cpp_detector::msg::Detection item;
            item.class_id = det.class_id;
            item.track_id = det.track_id;
            item.score = det.score;
            item.x1 = det.x1;
            item.y1 = det.y1;
            item.x2 = det.x2;
            item.y2 = det.y2;
            message.detections.push_back(item);
        }
        publisher_->publish(message);
        ++published_;
        if (track_log_.is_open()) {
            track_log_ << "{\"frame\":" << published_ << ",\"detections\":[";
            for (size_t i = 0; i < result.detections.size(); ++i) {
                const RawDetection &det = result.detections[i];
                if (i) track_log_ << ",";
                track_log_ << "{\"class_id\":" << det.class_id
                            << ",\"track_id\":" << det.track_id
                            << ",\"score\":" << std::setprecision(6) << det.score
                            << ",\"bbox\":[" << det.x1 << "," << det.y1 << ","
                            << det.x2 << "," << det.y2 << "]}";
            }
            track_log_ << "]}\n";
            track_log_.flush();
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "frame=%zu source=%s detections=%zu pre=%.2f infer=%.2f decode=%.2f total=%.2f ms",
            published_, image_path.c_str(), message.detections.size(),
            message.preprocess_ms, message.infer_ms, message.decode_ms,
            message.total_ms);
    }

    void tick() {
        if (finished_) return;
        const std::string &image_path = images_[index_];
        const FrameResult result = process_image(image_path);
        publish_result(result, image_path, now());
        if (csv_.is_open()) {
            csv_ << published_ << "," << image_path << "," << result.detections.size()
                 << "," << result.read_ms << "," << result.letterbox_ms << ","
                 << result.nv12_ms << "," << result.preprocess_ms << ","
                 << result.infer_ms << "," << result.decode_ms << ","
                 << result.total_ms << "\n";
            csv_.flush();
        }
        if (!dump_json_dir_.empty()) dump_frame_json(published_, image_path, result);
        ++index_;
        if (index_ >= images_.size()) {
            index_ = 0;
            if (!loop_) {
                finished_ = true;
                timer_->cancel();
                RCLCPP_INFO(get_logger(), "finished %zu frames", published_);
                if (csv_.is_open()) csv_.close();
                rclcpp::shutdown();
            }
        }
    }

    std::string model_path_;
    std::string image_dir_;
    std::string mode_;
    std::string image_topic_;
    double confidence_ = 0.25;
    double iou_ = 0.45;
    int top_k_ = 300;
    bool loop_ = true;
    std::vector<std::string> images_;
    size_t index_ = 0;
    size_t published_ = 0;
    bool finished_ = false;
    bool tracking_enabled_ = false;
    evi_tracking::MotionTracker tracker_;
    std::chrono::steady_clock::time_point last_tracking_time_{};
    std::string csv_path_;
    std::ofstream csv_;
    std::string dump_json_dir_;
    std::string track_log_path_;
    std::ofstream track_log_;
    std::string nv12_mode_;
    cv::Mat nv12_scratch_;
    float logit_threshold_ = 0.0f;

    hbPackedDNNHandle_t packed_ = nullptr;
    hbDNNHandle_t dnn_ = nullptr;
    hbDNNTensorProperties in_props_;
    int in_bytes_ = 0;
    int out_count_ = 0;
    std::vector<hbDNNTensor> outs_;
    hbDNNTensor input_;
    hbDNNInferCtrlParam control_;

    rclcpp::Publisher<evi_cpp_detector::msg::DetectionArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
