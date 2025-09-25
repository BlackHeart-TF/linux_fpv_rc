#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include "network/command_receiver.h"

namespace servo {

class I2CServoController {
public:
    struct Config {
        std::string i2c_device = "/dev/i2c-1";
        uint8_t controller_address = 0x40; // PCA9685 default address
        uint32_t pwm_frequency = 50; // 50Hz for standard servos
        uint16_t min_pulse_width = 500;  // 0.5ms minimum pulse
        uint16_t max_pulse_width = 2500; // 2.5ms maximum pulse
        uint16_t center_pulse_width = 1500; // 1.5ms center position
        bool enable_smooth_movement = true;
        uint8_t movement_speed = 10; // Steps per update for smooth movement
    };

    explicit I2CServoController(const Config& config);
    ~I2CServoController();

    // Non-copyable, movable
    I2CServoController(const I2CServoController&) = delete;
    I2CServoController& operator=(const I2CServoController&) = delete;
    I2CServoController(I2CServoController&&) = default;
    I2CServoController& operator=(I2CServoController&&) = default;

    bool initialize();
    void shutdown();

    // Servo control
    bool set_servo_position(uint8_t servo_id, uint16_t position, uint8_t speed = 255);
    bool set_multiple_servos(const std::vector<network::ServoPosition>& servos);
    
    // Get current positions
    uint16_t get_servo_position(uint8_t servo_id) const;
    std::array<uint16_t, MAX_SERVO_COUNT> get_all_positions() const;

    // Servo management
    bool enable_servo(uint8_t servo_id);
    bool disable_servo(uint8_t servo_id);
    void disable_all_servos();

    // Calibration
    bool calibrate_servo(uint8_t servo_id, uint16_t min_pos, uint16_t max_pos, uint16_t center_pos);

    struct ServoStatus {
        uint16_t current_position;
        uint16_t target_position;
        bool enabled;
        bool moving;
        uint16_t min_position;
        uint16_t max_position;
    };

    ServoStatus get_servo_status(uint8_t servo_id) const;

private:
    Config config_;
    int i2c_fd_;
    std::atomic<bool> initialized_;
    
    // Servo state tracking
    mutable std::mutex servo_mutex_;
    std::array<ServoStatus, MAX_SERVO_COUNT> servo_status_;
    
    // Smooth movement support
    std::atomic<bool> movement_active_;
    std::thread movement_thread_;
    
    bool open_i2c_device();
    bool setup_pwm_controller();
    bool write_i2c_register(uint8_t reg, uint8_t value);
    bool write_i2c_register_16(uint8_t reg, uint16_t value);
    bool read_i2c_register(uint8_t reg, uint8_t& value);
    
    // PWM calculations
    uint16_t position_to_pwm(uint16_t position, uint8_t servo_id) const;
    uint16_t pwm_to_position(uint16_t pwm_value, uint8_t servo_id) const;
    
    // PCA9685 specific functions
    bool set_pwm_frequency(uint32_t frequency);
    bool set_pwm_duty_cycle(uint8_t channel, uint16_t duty_cycle);
    
    // Smooth movement
    void movement_loop();
    void update_smooth_movement();
    
    void cleanup();
    
    // PCA9685 register definitions
    static constexpr uint8_t PCA9685_MODE1 = 0x00;
    static constexpr uint8_t PCA9685_MODE2 = 0x01;
    static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
    static constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
    static constexpr uint8_t PCA9685_LED0_ON_H = 0x07;
    static constexpr uint8_t PCA9685_LED0_OFF_L = 0x08;
    static constexpr uint8_t PCA9685_LED0_OFF_H = 0x09;
    static constexpr uint8_t PCA9685_ALL_LED_OFF_H = 0xFD;
};

} // namespace servo
