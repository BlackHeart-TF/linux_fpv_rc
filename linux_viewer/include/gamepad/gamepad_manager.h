#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>
#include <SDL2/SDL.h>

namespace gamepad {

struct ServoMapping {
    uint8_t servo_id;
    int axis;                   // SDL joystick axis index
    bool invert;               // Invert axis direction
    uint16_t min_value;        // Minimum servo position
    uint16_t max_value;        // Maximum servo position
    float deadzone;            // Per-axis deadzone override
};

struct ButtonMapping {
    int button;                // SDL button index
    std::string action;        // Action name (e.g., "center_servos", "force_iframe")
};

using ServoCommandCallback = std::function<void(uint8_t servo_id, uint16_t position, uint8_t speed)>;
using ButtonActionCallback = std::function<void(const std::string& action)>;

class GamepadManager {
public:
    struct Config {
        int device_index = 0;
        float global_deadzone = 0.1f;
        std::vector<ServoMapping> servo_mappings;
        std::vector<ButtonMapping> button_mappings;
        int update_rate_hz = 50;
        bool enable_rumble = false;
    };

    explicit GamepadManager(const Config& config);
    ~GamepadManager();

    // Non-copyable, movable
    GamepadManager(const GamepadManager&) = delete;
    GamepadManager& operator=(const GamepadManager&) = delete;
    GamepadManager(GamepadManager&&) = default;
    GamepadManager& operator=(GamepadManager&&) = default;

    bool initialize();
    void shutdown();
    bool is_initialized() const;

    // Control
    void start_input_processing();
    void stop_input_processing();
    bool is_processing() const;

    // Callbacks
    void set_servo_command_callback(ServoCommandCallback callback);
    void set_button_action_callback(ButtonActionCallback callback);

    // Gamepad info
    struct GamepadInfo {
        std::string name;
        int num_axes;
        int num_buttons;
        int num_hats;
        bool is_connected;
        SDL_JoystickID instance_id;
    };
    
    GamepadInfo get_gamepad_info() const;
    std::vector<GamepadInfo> get_available_gamepads() const;

    // Manual control
    bool set_servo_from_axis(int axis, float value);
    void center_all_servos();
    void trigger_button_action(const std::string& action);

    // Rumble support
    bool set_rumble(float low_frequency, float high_frequency, uint32_t duration_ms);

    // Statistics
    struct Stats {
        uint64_t packets_sent;
        uint64_t button_presses;
        uint64_t axis_updates;
        double update_rate;
        bool gamepad_connected;
    };
    
    Stats get_stats() const;

private:
    Config config_;
    std::atomic<bool> initialized_;
    std::atomic<bool> processing_;
    std::atomic<bool> should_stop_;
    
    // SDL components
    SDL_Joystick* joystick_;
    SDL_GameController* controller_;
    SDL_Haptic* haptic_;
    
    // Threading
    std::thread input_thread_;
    
    // Callbacks
    ServoCommandCallback servo_callback_;
    ButtonActionCallback button_callback_;
    std::mutex callback_mutex_;
    
    // State tracking
    std::vector<float> last_axis_values_;
    std::vector<bool> last_button_states_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    bool setup_gamepad();
    void input_loop();
    void process_axis_input();
    void process_button_input();
    void process_hat_input();
    
    float apply_deadzone(float value, float deadzone) const;
    uint16_t axis_to_servo_position(float axis_value, const ServoMapping& mapping) const;
    
    void update_stats();
    void cleanup();
    
    // SDL event handling
    void handle_controller_added(int device_index);
    void handle_controller_removed(SDL_JoystickID instance_id);
};

// Utility functions
std::vector<std::string> get_available_gamepad_names();
bool is_gamepad_connected(int device_index);

} // namespace gamepad
