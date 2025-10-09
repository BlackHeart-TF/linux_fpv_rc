#include "decoder/h264_decoder.h"
#include "utils/logger.h"

#include <chrono>
#include <cstring>

namespace decoder {

H264Decoder::H264Decoder(const Config& config)
    : config_(config), initialized_(false), codec_(nullptr), codec_context_(nullptr),
      frame_(nullptr), packet_(nullptr), sws_context_(nullptr), converted_frame_(nullptr),
      conversion_buffer_(nullptr), conversion_buffer_size_(0), should_stop_(false) {
    memset(&stats_, 0, sizeof(stats_));
    last_stats_update_ = std::chrono::steady_clock::now();
}

H264Decoder::~H264Decoder() {
    shutdown();
}

bool H264Decoder::initialize() {
    LOG_INFO("Initializing H.264 decoder");
    
    if (initialized_) {
        LOG_WARNING("Decoder already initialized");
        return true;
    }
    
    if (!setup_decoder()) {
        LOG_ERROR("Failed to setup decoder");
        return false;
    }
    
    // Start decoder thread
    should_stop_ = false;
    decoder_thread_ = std::thread(&H264Decoder::decoder_loop, this);
    
    initialized_ = true;
    LOG_INFO("H.264 decoder initialized successfully");
    return true;
}

void H264Decoder::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down H.264 decoder");
    
    // Stop decoder thread
    should_stop_ = true;
    queue_cv_.notify_all();
    
    if (decoder_thread_.joinable()) {
        decoder_thread_.join();
    }
    
    cleanup();
    initialized_ = false;
    LOG_INFO("H.264 decoder shutdown complete");
}

bool H264Decoder::is_initialized() const {
    return initialized_;
}

bool H264Decoder::decode_packet(const EncodedPacket& packet) {
    if (!initialized_) {
        return false;
    }
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Drop packets if queue is full
    if (packet_queue_.size() >= static_cast<size_t>(config_.max_frame_queue_size)) {
        packet_queue_.pop();
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.frames_dropped++;
    }
    
    packet_queue_.push(packet);
    queue_cv_.notify_one();
    
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.packets_received++;
    
    return true;
}

void H264Decoder::set_decoded_frame_callback(DecodedFrameCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
}

H264Decoder::Stats H264Decoder::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

std::string H264Decoder::get_decoder_info() {
    std::string info = "Available H.264 decoders:\n";
    
    const AVCodec* codec = nullptr;
    void* iter = nullptr;
    
    while ((codec = av_codec_iterate(&iter))) {
        if (codec->type == AVMEDIA_TYPE_VIDEO && codec->id == AV_CODEC_ID_H264) {
            info += "  - " + std::string(codec->name);
            if (codec->long_name) {
                info += " (" + std::string(codec->long_name) + ")";
            }
            info += "\n";
        }
    }
    
    return info;
}

bool H264Decoder::is_hardware_acceleration_available() {
    return false; // Simplified version without hardware acceleration
}

bool H264Decoder::setup_decoder() {
    // Find H.264 decoder
    codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec_) {
        LOG_ERROR("H.264 decoder not found");
        return false;
    }
    
    LOG_INFO("Using decoder: " + std::string(codec_->name));
    
    // Allocate codec context
    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
        LOG_ERROR("Failed to allocate codec context");
        return false;
    }
    
    // Configure decoder
    codec_context_->thread_count = config_.decoder_threads;
    codec_context_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
    
    // Use software decoder only for now
    if (!setup_software_decoder()) {
        return false;
    }
    
    // Allocate frame structures
    frame_ = av_frame_alloc();
    if (!frame_) {
        LOG_ERROR("Failed to allocate frame");
        return false;
    }
    
    packet_ = av_packet_alloc();
    if (!packet_) {
        LOG_ERROR("Failed to allocate packet");
        return false;
    }
    
    converted_frame_ = av_frame_alloc();
    if (!converted_frame_) {
        LOG_ERROR("Failed to allocate converted frame");
        return false;
    }
    
    return true;
}

bool H264Decoder::setup_hardware_acceleration() {
    return false; // Not implemented in simplified version
}

bool H264Decoder::init_hw_decoder(AVHWDeviceType type) {
    return false; // Not implemented in simplified version
}

bool H264Decoder::setup_software_decoder() {
    int ret = avcodec_open2(codec_context_, codec_, nullptr);
    if (ret < 0) {
        char error_buf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, error_buf, sizeof(error_buf));
        LOG_ERROR("Failed to open codec: " + std::string(error_buf));
        return false;
    }
    
    LOG_INFO("Software decoding enabled");
    return true;
}

AVPixelFormat H264Decoder::get_hw_format(AVCodecContext* ctx, const AVPixelFormat* pix_fmts) {
    return AV_PIX_FMT_NONE; // Not used in simplified version
}

void H264Decoder::decoder_loop() {
    LOG_INFO("Decoder thread started");
    
    while (!should_stop_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // Wait for packets or stop signal
        queue_cv_.wait(lock, [this] { return !packet_queue_.empty() || should_stop_; });
        
        if (should_stop_) {
            break;
        }
        
        if (packet_queue_.empty()) {
            continue;
        }
        
        // Get packet from queue
        EncodedPacket encoded_packet = packet_queue_.front();
        packet_queue_.pop();
        lock.unlock();
        
        // Process packet
        auto start_time = std::chrono::steady_clock::now();
        bool success = process_packet(encoded_packet);
        auto end_time = std::chrono::steady_clock::now();
        
        double decode_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        update_stats(success, decode_time_ms);
    }
    
    LOG_INFO("Decoder thread ended");
}

bool H264Decoder::process_packet(const EncodedPacket& encoded_packet) {
    LOG_INFO("Processing packet: size=" + std::to_string(encoded_packet.data.size()) + 
             ", keyframe=" + (encoded_packet.is_keyframe ? "true" : "false"));
    
    // Fill packet data
    packet_->data = const_cast<uint8_t*>(encoded_packet.data.data());
    packet_->size = encoded_packet.data.size();
    packet_->pts = encoded_packet.timestamp_us;
    
    // Send packet to decoder
    LOG_INFO("Sending packet to decoder...");
    int ret = avcodec_send_packet(codec_context_, packet_);
    if (ret < 0) {
        if (ret != AVERROR(EAGAIN)) {
            LOG_ERROR("Error sending packet to decoder: " + std::to_string(ret));
        }
        return false;
    }
    LOG_INFO("Packet sent to decoder successfully");
    
    // Receive decoded frames
    LOG_INFO("Starting to receive frames from decoder...");
    while (ret >= 0) {
        LOG_INFO("Calling avcodec_receive_frame...");
        ret = avcodec_receive_frame(codec_context_, frame_);
        LOG_INFO("avcodec_receive_frame returned: " + std::to_string(ret));
        
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            LOG_INFO("Decoder needs more data or EOF");
            break;
        }
        
        if (ret < 0) {
            LOG_ERROR("Error receiving frame from decoder: " + std::to_string(ret));
            return false;
        }
        
        // Convert and deliver frame
        DecodedFrame decoded_frame;
        if (convert_frame(frame_, decoded_frame)) {
            std::lock_guard<std::mutex> callback_lock(callback_mutex_);
            if (frame_callback_) {
                frame_callback_(decoded_frame);
            }
            
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.frames_decoded++;
        } else {
            LOG_ERROR("Failed to convert frame");
            return false;
        }
    }
    
    return true;
}

bool H264Decoder::convert_frame(AVFrame* src_frame, DecodedFrame& dst_frame) {
    // Set frame metadata
    dst_frame.width = src_frame->width;
    dst_frame.height = src_frame->height;
    dst_frame.timestamp_us = src_frame->pts;
    dst_frame.pixel_format = static_cast<AVPixelFormat>(src_frame->format);
    
    // Simple conversion - just copy YUV420P data directly
    if (src_frame->format == AV_PIX_FMT_YUV420P) {
        // Copy data pointers and line sizes
        for (int i = 0; i < 4; i++) {
            dst_frame.data[i] = src_frame->data[i];
            dst_frame.linesize[i] = src_frame->linesize[i];
        }
    } else {
        // Need to convert format
        if (!sws_context_) {
            sws_context_ = sws_getContext(
                src_frame->width, src_frame->height, static_cast<AVPixelFormat>(src_frame->format),
                src_frame->width, src_frame->height, config_.output_format,
                SWS_BILINEAR, nullptr, nullptr, nullptr);
                
            if (!sws_context_) {
                LOG_ERROR("Failed to create scaling context");
                return false;
            }
        }
        
        // Allocate conversion buffer if needed
        size_t required_size = av_image_get_buffer_size(config_.output_format, src_frame->width, src_frame->height, 1);
        if (conversion_buffer_size_ < required_size) {
            av_free(conversion_buffer_);
            conversion_buffer_ = static_cast<uint8_t*>(av_malloc(required_size));
            if (!conversion_buffer_) {
                LOG_ERROR("Failed to allocate conversion buffer");
                return false;
            }
            conversion_buffer_size_ = required_size;
        }
        
        // Setup converted frame
        av_image_fill_arrays(converted_frame_->data, converted_frame_->linesize,
                           conversion_buffer_, config_.output_format,
                           src_frame->width, src_frame->height, 1);
        
        // Convert frame
        sws_scale(sws_context_, src_frame->data, src_frame->linesize, 0, src_frame->height,
                 converted_frame_->data, converted_frame_->linesize);
        
        // Copy converted frame data pointers and line sizes
        for (int i = 0; i < 4; i++) {
            dst_frame.data[i] = converted_frame_->data[i];
            dst_frame.linesize[i] = converted_frame_->linesize[i];
        }
    }
    
    return true;
}

void H264Decoder::update_stats(bool decode_success, double decode_time_ms) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    if (!decode_success) {
        stats_.decode_errors++;
    }
    
    // Update average decode time (simple moving average)
    static constexpr double alpha = 0.1;
    stats_.average_decode_time_ms = alpha * decode_time_ms + (1.0 - alpha) * stats_.average_decode_time_ms;
    
    // Update FPS calculation
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_).count();
    
    if (elapsed >= 1000) { // Update every second
        static uint64_t last_frames = 0;
        uint64_t frames_diff = stats_.frames_decoded - last_frames;
        stats_.current_fps = frames_diff * 1000.0 / elapsed;
        
        last_stats_update_ = now;
        last_frames = stats_.frames_decoded;
    }
}

void H264Decoder::cleanup() {
    if (sws_context_) {
        sws_freeContext(sws_context_);
        sws_context_ = nullptr;
    }
    
    if (conversion_buffer_) {
        av_free(conversion_buffer_);
        conversion_buffer_ = nullptr;
        conversion_buffer_size_ = 0;
    }
    
    if (converted_frame_) {
        av_frame_free(&converted_frame_);
    }
    
    if (frame_) {
        av_frame_free(&frame_);
    }
    
    if (packet_) {
        av_packet_free(&packet_);
    }
    
    if (codec_context_) {
        avcodec_free_context(&codec_context_);
    }
    
    // Clear packet queue
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!packet_queue_.empty()) {
        packet_queue_.pop();
    }
}

} // namespace decoder
