#include "encoder/h264_encoder.h"
#include "utils/logger.h"
#include "utils/performance_monitor.h"

#include <cstring>

namespace encoder {

H264Encoder::H264Encoder(const Config& config)
    : config_(config), codec_(nullptr), codec_ctx_(nullptr), 
      frame_(nullptr), packet_(nullptr), sws_ctx_(nullptr),
      frame_count_(0), initialized_(false), force_iframe_(true) {
    memset(&stats_, 0, sizeof(stats_));
}

H264Encoder::~H264Encoder() {
    cleanup();
}

bool H264Encoder::initialize() {
    LOG_INFO("Initializing H.264 encoder");
    
    if (!setup_codec()) {
        LOG_ERROR("Failed to setup H.264 codec");
        return false;
    }
    
    initialized_ = true;
    LOG_INFO("H.264 encoder initialized successfully");
    return true;
}

void H264Encoder::set_encoded_frame_callback(EncodedFrameCallback callback) {
    encoded_callback_ = callback;
}

bool H264Encoder::encode_frame(const camera::FrameData& frame) {
    if (!initialized_) {
        LOG_ERROR("Encoder not initialized");
        return false;
    }
    
    MEASURE_ENCODE_TIME();
    
    // Convert input frame to encoder format
    if (!convert_frame(frame, frame_)) {
        LOG_ERROR("Failed to convert frame format");
        return false;
    }
    
    frame_->pts = frame_count_++;
    
    // Force I-frame if requested
    if (force_iframe_) {
        frame_->pict_type = AV_PICTURE_TYPE_I;
        frame_->flags |= AV_FRAME_FLAG_KEY;
        force_iframe_ = false;
        LOG_DEBUG("Forcing I-frame");
    } else {
        frame_->pict_type = AV_PICTURE_TYPE_NONE;  // Let encoder decide
        frame_->flags &= ~AV_FRAME_FLAG_KEY;
    }
    
    // Send frame to encoder
    int ret = avcodec_send_frame(codec_ctx_, frame_);
    if (ret < 0) {
        LOG_ERROR("Error sending frame to encoder: " + std::to_string(ret));
        return false;
    }
    
    // Receive encoded packets
    while (ret >= 0) {
        ret = avcodec_receive_packet(codec_ctx_, packet_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            LOG_ERROR("Error receiving packet from encoder: " + std::to_string(ret));
            return false;
        }
        
        // Create encoded frame data
        EncodedFrame encoded_frame;
        encoded_frame.data = packet_->data;
        encoded_frame.size = packet_->size;
        encoded_frame.timestamp_us = frame.timestamp_us;
        encoded_frame.is_keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            if (encoded_frame.is_keyframe) {
                stats_.iframes_encoded++;
                stats_.avg_iframe_size = (stats_.avg_iframe_size * (stats_.iframes_encoded - 1) + packet_->size) / stats_.iframes_encoded;
            } else {
                stats_.pframes_encoded++;
                stats_.avg_pframe_size = (stats_.avg_pframe_size * (stats_.pframes_encoded - 1) + packet_->size) / stats_.pframes_encoded;
            }
            stats_.total_bytes_encoded += packet_->size;
        }
        
        // Call callback if set
        if (encoded_callback_) {
            encoded_callback_(encoded_frame);
        }
        
        av_packet_unref(packet_);
    }
    
    return true;
}

void H264Encoder::flush() {
    if (!initialized_) {
        return;
    }
    
    // Send null frame to flush encoder
    avcodec_send_frame(codec_ctx_, nullptr);
    
    // Receive remaining packets
    int ret;
    while (ret >= 0) {
        ret = avcodec_receive_packet(codec_ctx_, packet_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            break;
        }
        
        EncodedFrame encoded_frame;
        encoded_frame.data = packet_->data;
        encoded_frame.size = packet_->size;
        encoded_frame.timestamp_us = 0;
        encoded_frame.is_keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
        
        if (encoded_callback_) {
            encoded_callback_(encoded_frame);
        }
        
        av_packet_unref(packet_);
    }
}

bool H264Encoder::setup_codec() {
    // Try hardware encoder first if enabled
    if (config_.use_hardware) {
        if (setup_hardware_encoder()) {
            LOG_INFO("Using hardware H.264 encoder");
            return true;
        }
        LOG_WARNING("Hardware encoder not available, falling back to software");
    }
    
    return setup_software_encoder();
}

bool H264Encoder::setup_hardware_encoder() {
#ifdef ENABLE_HARDWARE_ENCODING
    // Try various hardware encoders
    const char* hw_encoders[] = {
        "h264_v4l2m2m",    // Video4Linux2 Memory-to-Memory
        "h264_omx",        // OpenMAX IL
        "h264_nvenc",      // NVIDIA NVENC
        "h264_vaapi",      // Video Acceleration API
        nullptr
    };
    
    for (const char** encoder_name = hw_encoders; *encoder_name; encoder_name++) {
        codec_ = avcodec_find_encoder_by_name(*encoder_name);
        if (!codec_) {
            continue;
        }
        
        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            continue;
        }
        
        // Set codec parameters
        codec_ctx_->bit_rate = config_.bitrate;
        codec_ctx_->width = config_.width;
        codec_ctx_->height = config_.height;
        codec_ctx_->time_base = {1, static_cast<int>(config_.fps)};
        codec_ctx_->framerate = {static_cast<int>(config_.fps), 1};
        codec_ctx_->gop_size = config_.gop_size;
        codec_ctx_->max_b_frames = 0; // Disable B-frames for low latency
        codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
        
        // Set profile
        if (config_.profile == "baseline") {
            codec_ctx_->profile = FF_PROFILE_H264_BASELINE;
        } else if (config_.profile == "main") {
            codec_ctx_->profile = FF_PROFILE_H264_MAIN;
        }
        
        // Try to open codec
        if (avcodec_open2(codec_ctx_, codec_, nullptr) >= 0) {
            LOG_INFO("Hardware encoder initialized: " + std::string(*encoder_name));
            break;
        }
        
        avcodec_free_context(&codec_ctx_);
        codec_ = nullptr;
    }
    
    if (!codec_ctx_) {
        return false;
    }
#else
    return false;
#endif
    
    // Allocate frame and packet
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    
    if (!frame_ || !packet_) {
        LOG_ERROR("Failed to allocate frame or packet");
        return false;
    }
    
    frame_->format = codec_ctx_->pix_fmt;
    frame_->width = codec_ctx_->width;
    frame_->height = codec_ctx_->height;
    
    if (av_frame_get_buffer(frame_, 0) < 0) {
        LOG_ERROR("Failed to allocate frame buffer");
        return false;
    }
    
    return true;
}

bool H264Encoder::setup_software_encoder() {
    codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec_) {
        LOG_ERROR("H.264 encoder not found");
        return false;
    }
    
    codec_ctx_ = avcodec_alloc_context3(codec_);
    if (!codec_ctx_) {
        LOG_ERROR("Failed to allocate codec context");
        return false;
    }
    
    // Set codec parameters for I+P only, low latency
    codec_ctx_->bit_rate = config_.bitrate;
    codec_ctx_->width = config_.width;
    codec_ctx_->height = config_.height;
    codec_ctx_->time_base = {1, static_cast<int>(config_.fps)};
    codec_ctx_->framerate = {static_cast<int>(config_.fps), 1};
    codec_ctx_->gop_size = config_.gop_size;
    codec_ctx_->max_b_frames = 0; // No B-frames, only I and P
        codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;  // Standard format for software encoder
    
    // Set profile
    if (config_.profile == "baseline") {
        codec_ctx_->profile = FF_PROFILE_H264_BASELINE;
    } else if (config_.profile == "main") {
        codec_ctx_->profile = FF_PROFILE_H264_MAIN;
    }
    
    // Set codec options for ultra-low latency I+P only streaming
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "preset", config_.preset.c_str(), 0);
    av_dict_set(&opts, "tune", "zerolatency", 0);
    av_dict_set(&opts, "x264-params", 
                "bframes=0:force-cfr=1:no-scenecut=1:keyint=1000:min-keyint=1000:rc-lookahead=0", 0);
    av_dict_set(&opts, "flags", "+low_delay", 0);
    av_dict_set(&opts, "flags2", "+fast", 0);
    
    if (avcodec_open2(codec_ctx_, codec_, &opts) < 0) {
        LOG_ERROR("Failed to open codec");
        av_dict_free(&opts);
        return false;
    }
    
    av_dict_free(&opts);
    
    // Allocate frame and packet
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    
    if (!frame_ || !packet_) {
        LOG_ERROR("Failed to allocate frame or packet");
        return false;
    }
    
    frame_->format = codec_ctx_->pix_fmt;
    frame_->width = codec_ctx_->width;
    frame_->height = codec_ctx_->height;
    
    if (av_frame_get_buffer(frame_, 0) < 0) {
        LOG_ERROR("Failed to allocate frame buffer");
        return false;
    }
    
    LOG_INFO("Software H.264 encoder initialized");
    return true;
}

bool H264Encoder::convert_frame(const camera::FrameData& input, AVFrame* output) {
    // Validate input frame
    if (!input.data) {
        LOG_ERROR("Input frame data is null");
        return false;
    }
    
    size_t expected_size = config_.width * config_.height * 2; // YUYV is 2 bytes per pixel
    if (input.size < expected_size) {
        LOG_ERROR("Input frame size too small: " + std::to_string(input.size) + 
                  " expected: " + std::to_string(expected_size));
        return false;
    }
    
    // Setup SWS context for format conversion if needed
    if (!sws_ctx_) {
        sws_ctx_ = sws_getContext(
            config_.width, config_.height, AV_PIX_FMT_YUYV422, // Input format (YUYV from camera)
            config_.width, config_.height, AV_PIX_FMT_YUV420P, // Output format (YUV420P for encoder)
            SWS_BILINEAR, nullptr, nullptr, nullptr
        );
        
        if (!sws_ctx_) {
            LOG_ERROR("Failed to create SWS context");
            return false;
        }
        LOG_INFO("SWS context created for YUYV->YUV420P conversion");
    }
    
    // Prepare source frame data for YUYV (packed format)
    uint8_t* src_data[4] = {
        input.data,                                           // YUYV packed data
        nullptr, nullptr, nullptr
    };
    int src_linesize[4] = {
        static_cast<int>(config_.width * 2),                  // YUYV linesize (2 bytes per pixel)
        0, 0, 0
    };
    
    // Convert frame
    int ret = sws_scale(sws_ctx_, src_data, src_linesize, 0, config_.height,
                       output->data, output->linesize);
    
    if (ret != static_cast<int>(config_.height)) {
        LOG_ERROR("Frame conversion failed");
        return false;
    }
    
    return true;
}

void H264Encoder::cleanup() {
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }
    
    if (frame_) {
        av_frame_free(&frame_);
    }
    
    if (packet_) {
        av_packet_free(&packet_);
    }
    
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
    }
    
    initialized_ = false;
}

void H264Encoder::force_next_iframe() {
    force_iframe_ = true;
    LOG_INFO("Next frame will be forced as I-frame");
}

H264Encoder::Stats H264Encoder::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

} // namespace encoder
