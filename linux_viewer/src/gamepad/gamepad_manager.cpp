#include "gamepad/gamepad_manager.h"
#include "utils/logger.h"

#include <chrono>
#include <cmath>
#include <algorithm>

namespace gamepad {

GamepadManager::GamepadManager(const Config& config)
    : config_(config), initialized_(false), processing_(false), should_stop_(false),
      joystick_(nullptr), controller_(nullptr), haptic_(nullptr) {
    memset(&stats_, 0, sizeof(stats_));
    last_stats_update_ = std::chrono::steady_clock::now();
}

GamepadManager::~GamepadManager() {
    shutdown();
}

bool GamepadManager::initialize() {
    LOG_INFO("Initializing gamepad manager");
    
    if (initialized_) {
        LOG_WARNING("Gamepad manager already initialized");
        return true;
    }
    
    // Initialize SDL joystick subsystem
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) < 0) {
        LOG_ERROR("Failed to initialize SDL subsystems: " + std::string(SDL_GetError()));
        return false;
    }
    
    if (!setup_gamepad()) {
        LOG_ERROR("Failed to setup gamepad");
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);
        return false;
    }
    
    initialized_ = true;
    LOG_INFO("Gamepad manager initialized successfully");
    return true;
}

void GamepadManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down gamepad manager");
    
    stop_input_processing();
    cleanup();
    
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);
    
    initialized_ = false;
    LOG_INFO("Gamepad manager shutdown complete");
}

bool GamepadManager::is_initialized() const {
    return initialized_;
}

void GamepadManager::start_input_processing() {
    if (!initialized_ || processing_) {
        return;
    }
    
    should_stop_ = false;
    input_thread_ = std::thread(&GamepadManager::input_loop, this);
    processing_ = true;
    
    LOG_INFO("Gamepad input processing started");
}

void GamepadManager::stop_input_processing() {
    if (!processing_) {
        return;
    }
    
    LOG_INFO("Stopping gamepad input processing");
    should_stop_ = true;
    
    if (input_thread_.joinable()) {
        input_thread_.join();
    }
    
    processing_ = false;
    LOG_INFO("Gamepad input processing stopped");
}

bool GamepadManager::is_processing() const {
    return processing_;
}

void GamepadManager::set_servo_command_callback(ServoCommandCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    servo_callback_ = callback;
}

void GamepadManager::set_button_action_callback(ButtonActionCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    button_callback_ = callback;
}

GamepadManager::GamepadInfo GamepadManager::get_gamepad_info() const {
    GamepadInfo info;
    
    if (joystick_) {
        info.name = SDL_JoystickName(joystick_);
        info.num_axes = SDL_JoystickNumAxes(joystick_);
        info.num_buttons = SDL_JoystickNumButtons(joystick_);
        info.num_hats = SDL_JoystickNumHats(joystick_);
        info.is_connected = SDL_JoystickGetAttached(joystick_);
        info.instance_id = SDL_JoystickInstanceID(joystick_);
    } else {
        info.name = "No gamepad connected";
        info.num_axes = 0;
        info.num_buttons = 0;
        info.num_hats = 0;
        info.is_connected = false;
        info.instance_id = -1;
    }
    
    return info;
}

std::vector<GamepadManager::GamepadInfo> GamepadManager::get_available_gamepads() const {
    std::vector<GamepadInfo> gamepads;
    
    int num_joysticks = SDL_NumJoysticks();
    for (int i = 0; i < num_joysticks; i++) {
        GamepadInfo info;
        
        if (SDL_IsGameController(i)) {
            SDL_GameController* controller = SDL_GameControllerOpen(i);
            if (controller) {
                SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
                info.name = SDL_GameControllerName(controller);
                info.num_axes = SDL_JoystickNumAxes(joystick);
                info.num_buttons = SDL_JoystickNumButtons(joystick);
                info.num_hats = SDL_JoystickNumHats(joystick);
                info.is_connected = true;
                info.instance_id = SDL_JoystickInstanceID(joystick);
                SDL_GameControllerClose(controller);
            }
        } else {
            SDL_Joystick* joystick = SDL_JoystickOpen(i);
            if (joystick) {
                info.name = SDL_JoystickName(joystick);
                info.num_axes = SDL_JoystickNumAxes(joystick);
                info.num_buttons = SDL_JoystickNumButtons(joystick);
                info.num_hats = SDL_JoystickNumHats(joystick);
                info.is_connected = true;
                info.instance_id = SDL_JoystickInstanceID(joystick);
                SDL_JoystickClose(joystick);
            }
        }
        
        gamepads.push_back(info);
    }
    
    return gamepads;
}

bool GamepadManager::set_servo_from_axis(int axis, float value) {
    if (!initialized_) {
        return false;
    }
    
    // Find servo mapping for this axis
    for (const auto& mapping : config_.servo_mappings) {
        if (mapping.axis == axis) {
            float deadzone = (mapping.deadzone > 0) ? mapping.deadzone : config_.global_deadzone;
            float processed_value = apply_deadzone(value, deadzone);
            
            if (mapping.invert) {
                processed_value = -processed_value;
            }
            
            uint16_t position = axis_to_servo_position(processed_value, mapping);
            
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (servo_callback_) {
                servo_callback_(mapping.servo_id, position, 255);
                
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                stats_.packets_sent++;
            }
            return true;
        }
    }
    
    return false;
}

void GamepadManager::center_all_servos() {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!servo_callback_) {
        return;
    }
    
    for (const auto& mapping : config_.servo_mappings) {
        uint16_t center_position = (mapping.min_value + mapping.max_value) / 2;
        servo_callback_(mapping.servo_id, center_position, 255);
        
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.packets_sent++;
    }
    
    LOG_INFO("Centered all servos");
}

void GamepadManager::trigger_button_action(const std::string& action) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (button_callback_) {
        button_callback_(action);
        
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.button_presses++;
    }
}

bool GamepadManager::set_rumble(float low_frequency, float high_frequency, uint32_t duration_ms) {
    if (!haptic_ || !config_.enable_rumble) {
        return false;
    }
    
    // Convert 0.0-1.0 range to 0-65535
    uint16_t low = static_cast<uint16_t>(std::clamp(low_frequency, 0.0f, 1.0f) * 65535);
    uint16_t high = static_cast<uint16_t>(std::clamp(high_frequency, 0.0f, 1.0f) * 65535);
    
    return SDL_HapticRumblePlay(haptic_, std::max(low_frequency, high_frequency), duration_ms) == 0;
}

GamepadManager::Stats GamepadManager::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    Stats stats = stats_;
    stats.gamepad_connected = (joystick_ != nullptr) && SDL_JoystickGetAttached(joystick_);
    return stats;
}

bool GamepadManager::setup_gamepad() {
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks == 0) {
        LOG_WARNING("No joysticks/gamepads detected");
        return true; // Not an error, just no gamepad available
    }
    
    LOG_INFO("Found " + std::to_string(num_joysticks) + " joystick(s)");
    
    if (config_.device_index >= num_joysticks) {
        LOG_ERROR("Gamepad device index " + std::to_string(config_.device_index) + 
                 " is out of range (0-" + std::to_string(num_joysticks - 1) + ")");
        return false;
    }
    
    // Try to open as game controller first (preferred)
    if (SDL_IsGameController(config_.device_index)) {
        controller_ = SDL_GameControllerOpen(config_.device_index);
        if (controller_) {
            joystick_ = SDL_GameControllerGetJoystick(controller_);
            LOG_INFO("Opened game controller: " + std::string(SDL_GameControllerName(controller_)));
        }
    }
    
    // Fallback to joystick interface
    if (!joystick_) {
        joystick_ = SDL_JoystickOpen(config_.device_index);
        if (joystick_) {
            LOG_INFO("Opened joystick: " + std::string(SDL_JoystickName(joystick_)));
        }
    }
    
    if (!joystick_) {
        LOG_ERROR("Failed to open gamepad device " + std::to_string(config_.device_index) + 
                 ": " + std::string(SDL_GetError()));
        return false;
    }
    
    // Initialize haptic feedback if available and enabled
    if (config_.enable_rumble && SDL_JoystickIsHaptic(joystick_)) {
        haptic_ = SDL_HapticOpenFromJoystick(joystick_);
        if (haptic_) {
            if (SDL_HapticRumbleInit(haptic_) == 0) {
                LOG_INFO("Haptic feedback initialized");
            } else {
                LOG_WARNING("Failed to initialize haptic rumble: " + std::string(SDL_GetError()));
                SDL_HapticClose(haptic_);
                haptic_ = nullptr;
            }
        }
    }
    
    // Initialize state tracking arrays
    int num_axes = SDL_JoystickNumAxes(joystick_);
    int num_buttons = SDL_JoystickNumButtons(joystick_);
    
    last_axis_values_.resize(num_axes, 0.0f);
    last_button_states_.resize(num_buttons, false);
    
    LOG_INFO("Gamepad setup complete - Axes: " + std::to_string(num_axes) + 
             ", Buttons: " + std::to_string(num_buttons));
    
    return true;
}

void GamepadManager::input_loop() {
    LOG_INFO("Gamepad input loop started");
    
    auto last_update = std::chrono::steady_clock::now();
    auto update_interval = std::chrono::milliseconds(1000 / config_.update_rate_hz);
    
    while (!should_stop_) {
        auto now = std::chrono::steady_clock::now();
        
        // Process SDL events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_CONTROLLERDEVICEADDED:
                    handle_controller_added(event.cdevice.which);
                    break;
                case SDL_CONTROLLERDEVICEREMOVED:
                    handle_controller_removed(event.cdevice.which);
                    break;
                case SDL_QUIT:
                    should_stop_ = true;
                    break;
            }
        }
        
        // Update inputs at specified rate
        if (now - last_update >= update_interval) {
            if (joystick_ && SDL_JoystickGetAttached(joystick_)) {
                process_axis_input();
                process_button_input();
                process_hat_input();
            }
            
            update_stats();
            last_update = now;
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    LOG_INFO("Gamepad input loop ended");
}

void GamepadManager::process_axis_input() {
    int num_axes = SDL_JoystickNumAxes(joystick_);
    
    for (int axis = 0; axis < num_axes; axis++) {
        Sint16 raw_value = SDL_JoystickGetAxis(joystick_, axis);
        float normalized_value = raw_value / 32767.0f; // Convert to -1.0 to 1.0 range
        
        // Check if value changed significantly
        if (std::abs(normalized_value - last_axis_values_[axis]) > 0.01f) {
            last_axis_values_[axis] = normalized_value;
            
            if (set_servo_from_axis(axis, normalized_value)) {
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                stats_.axis_updates++;
            }
        }
    }
}

void GamepadManager::process_button_input() {
    int num_buttons = SDL_JoystickNumButtons(joystick_);
    
    for (int button = 0; button < num_buttons; button++) {
        bool current_state = SDL_JoystickGetButton(joystick_, button) != 0;
        bool last_state = last_button_states_[button];
        
        // Detect button press (transition from false to true)
        if (current_state && !last_state) {
            // Find button mapping
            for (const auto& mapping : config_.button_mappings) {
                if (mapping.button == button) {
                    trigger_button_action(mapping.action);
                    LOG_DEBUG("Button " + std::to_string(button) + " pressed, action: " + mapping.action);
                    break;
                }
            }
        }
        
        last_button_states_[button] = current_state;
    }
}

void GamepadManager::process_hat_input() {
    int num_hats = SDL_JoystickNumHats(joystick_);
    
    for (int hat = 0; hat < num_hats; hat++) {
        Uint8 hat_state = SDL_JoystickGetHat(joystick_, hat);
        
        // Convert hat state to axis-like values for servo control
        float x_axis = 0.0f, y_axis = 0.0f;
        
        if (hat_state & SDL_HAT_LEFT) x_axis = -1.0f;
        else if (hat_state & SDL_HAT_RIGHT) x_axis = 1.0f;
        
        if (hat_state & SDL_HAT_UP) y_axis = 1.0f;
        else if (hat_state & SDL_HAT_DOWN) y_axis = -1.0f;
        
        // Map hat to virtual axes (hat 0 -> axes 100+101, hat 1 -> axes 102+103, etc.)
        int virtual_x_axis = 100 + (hat * 2);
        int virtual_y_axis = 100 + (hat * 2) + 1;
        
        set_servo_from_axis(virtual_x_axis, x_axis);
        set_servo_from_axis(virtual_y_axis, y_axis);
    }
}

float GamepadManager::apply_deadzone(float value, float deadzone) const {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }
    
    // Scale the remaining range to full range
    float sign = (value >= 0) ? 1.0f : -1.0f;
    float abs_value = std::abs(value);
    return sign * (abs_value - deadzone) / (1.0f - deadzone);
}

uint16_t GamepadManager::axis_to_servo_position(float axis_value, const ServoMapping& mapping) const {
    // Convert -1.0 to 1.0 range to servo position range
    float normalized = (axis_value + 1.0f) / 2.0f; // Convert to 0.0-1.0 range
    normalized = std::clamp(normalized, 0.0f, 1.0f);
    
    uint16_t position = static_cast<uint16_t>(
        mapping.min_value + normalized * (mapping.max_value - mapping.min_value)
    );
    
    return std::clamp(position, mapping.min_value, mapping.max_value);
}

void GamepadManager::update_stats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_).count();
    
    if (elapsed >= 1000) { // Update every second
        static uint64_t last_packets = 0;
        uint64_t packets_diff = stats_.packets_sent - last_packets;
        stats_.update_rate = packets_diff * 1000.0 / elapsed;
        
        last_stats_update_ = now;
        last_packets = stats_.packets_sent;
    }
}

void GamepadManager::handle_controller_added(int device_index) {
    LOG_INFO("Game controller added: " + std::to_string(device_index));
    
    if (device_index == config_.device_index && !joystick_) {
        // Try to reconnect to our configured device
        setup_gamepad();
    }
}

void GamepadManager::handle_controller_removed(SDL_JoystickID instance_id) {
    LOG_INFO("Game controller removed: " + std::to_string(instance_id));
    
    if (joystick_ && SDL_JoystickInstanceID(joystick_) == instance_id) {
        cleanup();
    }
}

void GamepadManager::cleanup() {
    if (haptic_) {
        SDL_HapticClose(haptic_);
        haptic_ = nullptr;
    }
    
    if (controller_) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
        joystick_ = nullptr; // Joystick is closed automatically with controller
    } else if (joystick_) {
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
    }
    
    last_axis_values_.clear();
    last_button_states_.clear();
}

// Utility functions
std::vector<std::string> get_available_gamepad_names() {
    std::vector<std::string> names;
    
    int num_joysticks = SDL_NumJoysticks();
    for (int i = 0; i < num_joysticks; i++) {
        if (SDL_IsGameController(i)) {
            names.push_back(SDL_GameControllerNameForIndex(i));
        } else {
            names.push_back(SDL_JoystickNameForIndex(i));
        }
    }
    
    return names;
}

bool is_gamepad_connected(int device_index) {
    return device_index >= 0 && device_index < SDL_NumJoysticks();
}

} // namespace gamepad
