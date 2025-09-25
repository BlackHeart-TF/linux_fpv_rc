#include "servo/i2c_servo_controller.h"
#include "utils/logger.h"

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

namespace servo {

I2CServoController::I2CServoController(const Config& config)
    : config_(config), i2c_fd_(-1), initialized_(false), movement_active_(false) {
    
    // Initialize servo status array
    for (auto& status : servo_status_) {
        status.current_position = config_.center_pulse_width;
        status.target_position = config_.center_pulse_width;
        status.enabled = false;
        status.moving = false;
        status.min_position = config_.min_pulse_width;
        status.max_position = config_.max_pulse_width;
    }
}

I2CServoController::~I2CServoController() {
    shutdown();
}

bool I2CServoController::initialize() {
    LOG_INFO("Initializing I2C servo controller on device: " + config_.i2c_device);
    
    if (!open_i2c_device()) {
        LOG_ERROR("Failed to open I2C device");
        return false;
    }
    
    if (!setup_pwm_controller()) {
        LOG_ERROR("Failed to setup PWM controller");
        return false;
    }
    
    // Start smooth movement thread if enabled
    if (config_.enable_smooth_movement) {
        movement_active_ = true;
        movement_thread_ = std::thread(&I2CServoController::movement_loop, this);
    }
    
    initialized_ = true;
    LOG_INFO("I2C servo controller initialized successfully");
    return true;
}

void I2CServoController::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down servo controller");
    
    // Disable all servos
    disable_all_servos();
    
    // Stop movement thread
    if (movement_active_) {
        movement_active_ = false;
        if (movement_thread_.joinable()) {
            movement_thread_.join();
        }
    }
    
    cleanup();
    initialized_ = false;
    LOG_INFO("Servo controller shutdown complete");
}

bool I2CServoController::set_servo_position(uint8_t servo_id, uint16_t position, uint8_t speed) {
    if (!initialized_ || servo_id >= MAX_SERVO_COUNT) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    // Clamp position to valid range
    auto& status = servo_status_[servo_id];
    position = std::max(status.min_position, std::min(status.max_position, position));
    
    status.target_position = position;
    status.enabled = true;
    
    if (!config_.enable_smooth_movement) {
        // Immediate movement
        status.current_position = position;
        status.moving = false;
        
        uint16_t pwm_value = position_to_pwm(position, servo_id);
        return set_pwm_duty_cycle(servo_id, pwm_value);
    } else {
        // Smooth movement will be handled by movement thread
        status.moving = (status.current_position != status.target_position);
        return true;
    }
}

bool I2CServoController::set_multiple_servos(const std::vector<network::ServoPosition>& servos) {
    if (!initialized_) {
        return false;
    }
    
    bool all_success = true;
    
    for (const auto& servo : servos) {
        if (servo.servo_id < MAX_SERVO_COUNT) {
            if (!set_servo_position(servo.servo_id, servo.position, 255)) {  // Default speed
                all_success = false;
                LOG_WARNING("Failed to set position for servo " + std::to_string(servo.servo_id));
            }
        }
    }
    
    return all_success;
}

uint16_t I2CServoController::get_servo_position(uint8_t servo_id) const {
    if (servo_id >= MAX_SERVO_COUNT) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    return servo_status_[servo_id].current_position;
}

std::array<uint16_t, MAX_SERVO_COUNT> I2CServoController::get_all_positions() const {
    std::array<uint16_t, MAX_SERVO_COUNT> positions;
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    for (size_t i = 0; i < MAX_SERVO_COUNT; i++) {
        positions[i] = servo_status_[i].current_position;
    }
    
    return positions;
}

bool I2CServoController::enable_servo(uint8_t servo_id) {
    if (!initialized_ || servo_id >= MAX_SERVO_COUNT) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_status_[servo_id].enabled = true;
    
    // Set servo to current position to enable it
    uint16_t pwm_value = position_to_pwm(servo_status_[servo_id].current_position, servo_id);
    return set_pwm_duty_cycle(servo_id, pwm_value);
}

bool I2CServoController::disable_servo(uint8_t servo_id) {
    if (!initialized_ || servo_id >= MAX_SERVO_COUNT) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_status_[servo_id].enabled = false;
    servo_status_[servo_id].moving = false;
    
    // Set PWM to 0 to disable servo
    return set_pwm_duty_cycle(servo_id, 0);
}

void I2CServoController::disable_all_servos() {
    for (uint8_t i = 0; i < MAX_SERVO_COUNT; i++) {
        disable_servo(i);
    }
}

bool I2CServoController::calibrate_servo(uint8_t servo_id, uint16_t min_pos, uint16_t max_pos, uint16_t center_pos) {
    if (!initialized_ || servo_id >= MAX_SERVO_COUNT) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    auto& status = servo_status_[servo_id];
    
    status.min_position = min_pos;
    status.max_position = max_pos;
    
    // Set to center position
    status.current_position = center_pos;
    status.target_position = center_pos;
    
    LOG_INFO("Calibrated servo " + std::to_string(servo_id) + 
             " - Range: " + std::to_string(min_pos) + "-" + std::to_string(max_pos) +
             " Center: " + std::to_string(center_pos));
    
    return true;
}

I2CServoController::ServoStatus I2CServoController::get_servo_status(uint8_t servo_id) const {
    if (servo_id >= MAX_SERVO_COUNT) {
        return {};
    }
    
    std::lock_guard<std::mutex> lock(servo_mutex_);
    return servo_status_[servo_id];
}

bool I2CServoController::open_i2c_device() {
    i2c_fd_ = open(config_.i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        LOG_ERROR("Failed to open I2C device " + config_.i2c_device + ": " + strerror(errno));
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, config_.controller_address) < 0) {
        LOG_ERROR("Failed to set I2C slave address: " + std::string(strerror(errno)));
        return false;
    }
    
    LOG_INFO("I2C device opened: " + config_.i2c_device + 
             " Address: 0x" + std::to_string(config_.controller_address));
    return true;
}

bool I2CServoController::setup_pwm_controller() {
    // Reset PCA9685
    if (!write_i2c_register(PCA9685_MODE1, 0x80)) {
        LOG_ERROR("Failed to reset PCA9685");
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Set PWM frequency
    if (!set_pwm_frequency(config_.pwm_frequency)) {
        LOG_ERROR("Failed to set PWM frequency");
        return false;
    }
    
    // Configure MODE1 register
    uint8_t mode1 = 0x01; // Enable auto-increment, normal mode
    if (!write_i2c_register(PCA9685_MODE1, mode1)) {
        LOG_ERROR("Failed to configure MODE1 register");
        return false;
    }
    
    // Configure MODE2 register
    uint8_t mode2 = 0x04; // Output drive = totem pole
    if (!write_i2c_register(PCA9685_MODE2, mode2)) {
        LOG_ERROR("Failed to configure MODE2 register");
        return false;
    }
    
    // Turn off all PWM outputs initially
    if (!write_i2c_register_16(PCA9685_ALL_LED_OFF_H, 0x1000)) {
        LOG_ERROR("Failed to turn off all PWM outputs");
        return false;
    }
    
    LOG_INFO("PCA9685 PWM controller configured - Frequency: " + std::to_string(config_.pwm_frequency) + "Hz");
    return true;
}

bool I2CServoController::write_i2c_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    
    if (write(i2c_fd_, buffer, 2) != 2) {
        LOG_ERROR("Failed to write I2C register 0x" + std::to_string(reg) + ": " + strerror(errno));
        return false;
    }
    
    return true;
}

bool I2CServoController::write_i2c_register_16(uint8_t reg, uint16_t value) {
    uint8_t buffer[3] = {reg, static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
    
    if (write(i2c_fd_, buffer, 3) != 3) {
        LOG_ERROR("Failed to write I2C register 16-bit 0x" + std::to_string(reg) + ": " + strerror(errno));
        return false;
    }
    
    return true;
}

bool I2CServoController::read_i2c_register(uint8_t reg, uint8_t& value) {
    if (write(i2c_fd_, &reg, 1) != 1) {
        LOG_ERROR("Failed to write register address: " + std::string(strerror(errno)));
        return false;
    }
    
    if (read(i2c_fd_, &value, 1) != 1) {
        LOG_ERROR("Failed to read register value: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

uint16_t I2CServoController::position_to_pwm(uint16_t position, uint8_t servo_id) const {
    // Convert position (pulse width in microseconds) to PWM value
    // PCA9685 has 12-bit resolution (0-4095)
    // At 50Hz, each tick is ~4.88μs (20000μs / 4096)
    
    const double tick_duration_us = 20000.0 / 4096.0; // ~4.88μs per tick at 50Hz
    uint16_t pwm_value = static_cast<uint16_t>(position / tick_duration_us);
    
    // Clamp to valid range
    return std::min(static_cast<uint16_t>(4095), pwm_value);
}

uint16_t I2CServoController::pwm_to_position(uint16_t pwm_value, uint8_t servo_id) const {
    const double tick_duration_us = 20000.0 / 4096.0;
    return static_cast<uint16_t>(pwm_value * tick_duration_us);
}

bool I2CServoController::set_pwm_frequency(uint32_t frequency) {
    // Calculate prescaler value
    // PCA9685 internal oscillator frequency is 25MHz
    // Prescaler = (25MHz / (4096 * frequency)) - 1
    
    uint8_t prescaler = static_cast<uint8_t>((25000000.0 / (4096.0 * frequency)) - 1 + 0.5);
    
    // Read current MODE1 register
    uint8_t old_mode1;
    if (!read_i2c_register(PCA9685_MODE1, old_mode1)) {
        return false;
    }
    
    // Put PCA9685 to sleep
    uint8_t new_mode1 = (old_mode1 & 0x7F) | 0x10; // Sleep mode
    if (!write_i2c_register(PCA9685_MODE1, new_mode1)) {
        return false;
    }
    
    // Set prescaler
    if (!write_i2c_register(PCA9685_PRESCALE, prescaler)) {
        return false;
    }
    
    // Restore old mode
    if (!write_i2c_register(PCA9685_MODE1, old_mode1)) {
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    // Enable auto-increment
    if (!write_i2c_register(PCA9685_MODE1, old_mode1 | 0x80)) {
        return false;
    }
    
    return true;
}

bool I2CServoController::set_pwm_duty_cycle(uint8_t channel, uint16_t duty_cycle) {
    if (channel >= 16) { // PCA9685 has 16 channels
        return false;
    }
    
    uint8_t reg_base = PCA9685_LED0_ON_L + (channel * 4);
    
    // Set ON time to 0
    if (!write_i2c_register(reg_base, 0)) return false;
    if (!write_i2c_register(reg_base + 1, 0)) return false;
    
    // Set OFF time
    if (duty_cycle == 0) {
        // Full OFF
        if (!write_i2c_register(reg_base + 2, 0)) return false;
        if (!write_i2c_register(reg_base + 3, 0x10)) return false;
    } else if (duty_cycle >= 4095) {
        // Full ON
        if (!write_i2c_register(reg_base + 2, 0)) return false;
        if (!write_i2c_register(reg_base + 3, 0x10)) return false;
    } else {
        // Normal PWM
        if (!write_i2c_register(reg_base + 2, duty_cycle & 0xFF)) return false;
        if (!write_i2c_register(reg_base + 3, (duty_cycle >> 8) & 0x0F)) return false;
    }
    
    return true;
}

void I2CServoController::movement_loop() {
    LOG_INFO("Servo smooth movement loop started");
    
    while (movement_active_) {
        update_smooth_movement();
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz update rate
    }
    
    LOG_INFO("Servo smooth movement loop ended");
}

void I2CServoController::update_smooth_movement() {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    for (uint8_t i = 0; i < MAX_SERVO_COUNT; i++) {
        auto& status = servo_status_[i];
        
        if (!status.enabled || !status.moving) {
            continue;
        }
        
        int16_t diff = status.target_position - status.current_position;
        if (abs(diff) <= config_.movement_speed) {
            // Close enough, snap to target
            status.current_position = status.target_position;
            status.moving = false;
        } else {
            // Move towards target
            if (diff > 0) {
                status.current_position += config_.movement_speed;
            } else {
                status.current_position -= config_.movement_speed;
            }
        }
        
        // Update PWM output
        uint16_t pwm_value = position_to_pwm(status.current_position, i);
        set_pwm_duty_cycle(i, pwm_value);
    }
}

void I2CServoController::cleanup() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

} // namespace servo
