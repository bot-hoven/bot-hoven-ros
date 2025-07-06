#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <fstream>
#include <sstream>
#include <queue>

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

/**
 * @brief Event struct to represent a single MIDI-like event
 */
struct Event {
    std::string note_name;    // Note name (e.g., C4, G3)
    std::string finger;       // Finger to use (thumb, index, middle, ring, pinky)
    std::string hand;         // Hand to use (left, right)
    std::string component;    // Component to control (servo or solenoid)
    double position;          // Position value (angle for servo, 0/1 for solenoid)
    double delta_time;        // Time in seconds since the last event
    
    // Absolute time when this event should occur (calculated during playback)
    rclcpp::Time scheduled_time;
};

/**
 * @brief Song struct to represent a full musical piece
 */
struct Song {
    std::string name;         // Song name
    double tempo_bpm;         // Tempo in beats per minute
    std::vector<Event> events; // All events in the song
};

/**
 * @brief ConfigurableSongPlayer class
 * 
 * A ROS2 node that can load songs from CSV files and play them
 * by sending commands to the robot hands
 */
class ConfigurableSongPlayer : public rclcpp::Node {
public:
    ConfigurableSongPlayer() : Node("configurable_song_player") {
        // Declare parameters
        this->declare_parameter("songs_directory", "songs");
        
        // Initialize action clients for servo controllers
        left_servo_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/left_servo_controller/follow_joint_trajectory"
        );

        right_servo_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/right_servo_controller/follow_joint_trajectory"
        );

        // Initialize publishers for solenoid controllers
        left_solenoid_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_solenoid_controller/commands", 10
        );

        right_solenoid_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_solenoid_controller/commands", 10
        );

        // Create a timer to check when clients are ready
        check_clients_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ConfigurableSongPlayer::check_clients_ready, this)
        );

        // Subscribe to song commands
        song_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "song_command", 10,
            std::bind(&ConfigurableSongPlayer::song_command_callback, this, std::placeholders::_1)
        );

        // Define finger mappings
        initialize_finger_joint_maps();

        RCLCPP_INFO(this->get_logger(), "Event-based ConfigurableSongPlayer initialized");
    }

    /**
     * @brief Load a song from a CSV file
     * 
     * Expected CSV format:
     * note_name,finger,hand,component,position,delta_time
     * 
     * @param filename The path to the CSV file
     * @return true if loading was successful, false otherwise
     */
    bool load_song_from_file(const std::string& filename) {
        std::string songs_dir = this->get_parameter("songs_directory").as_string();
        std::string filepath = songs_dir + "/" + filename;
        
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open song file: %s", filepath.c_str());
            return false;
        }
        
        Song new_song;
        new_song.name = filename;  // Use filename as song name initially
        
        // First line may contain metadata
        std::string line;
        if (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string key, value;
            
            // Check if this is metadata
            if (line.find('=') != std::string::npos) {
                while (std::getline(ss, key, '=')) {
                    std::getline(ss, value, ',');
                    
                    // Trim whitespace
                    key.erase(0, key.find_first_not_of(" \t"));
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    
                    if (key == "name") {
                        new_song.name = value;
                    } else if (key == "tempo") {
                        try {
                            new_song.tempo_bpm = std::stod(value);
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(this->get_logger(), "Invalid tempo value: %s", value.c_str());
                            new_song.tempo_bpm = 100.0;  // Default tempo
                        }
                    }
                }
            } else {
                // If not metadata, rewind to the beginning of the file
                file.seekg(0);
            }
        }
        
        // If tempo wasn't set in metadata, use default
        if (new_song.tempo_bpm <= 0.0) {
            new_song.tempo_bpm = 120.0;  // Default tempo 
        }
        
        // Process events
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;  // Skip empty lines and comments
            
            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            
            while (std::getline(ss, token, ',')) {
                // Trim whitespace
                token.erase(0, token.find_first_not_of(" \t"));
                token.erase(token.find_last_not_of(" \t") + 1);
                tokens.push_back(token);
            }
            
            // Expect 6 columns: note_name, finger, hand, component, position, delta_time
            if (tokens.size() < 6) {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
                continue;
            }
            
            Event event;
            event.note_name = tokens[0];
            event.finger = tokens[1];
            event.hand = tokens[2];
            event.component = tokens[3];
            
            try {
                event.position = std::stod(tokens[4]);
                event.delta_time = std::stod(tokens[5]);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Invalid numeric value in line: %s", line.c_str());
                continue;
            }
            
            // Validate the component type
            if (event.component != "servo" && event.component != "solenoid") {
                RCLCPP_WARN(this->get_logger(), "Invalid component type: %s (must be 'servo' or 'solenoid')",
                           event.component.c_str());
                continue;
            }
            
            new_song.events.push_back(event);
        }
        
        if (new_song.events.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid events found in song file: %s", filepath.c_str());
            return false;
        }
        
        // Store the loaded song
        current_song_ = new_song;
        RCLCPP_INFO(this->get_logger(), "Successfully loaded song: %s (Tempo: %.1f BPM, Events: %zu)",
                   current_song_.name.c_str(), current_song_.tempo_bpm, current_song_.events.size());
        
        return true;
    }

    /**
     * @brief Play the currently loaded song
     */
    void play() {
        if (current_song_.events.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No song loaded. Please load a song first.");
            return;
        }

        if (!left_servo_client_ready_ || !right_servo_client_ready_) {
            RCLCPP_ERROR(this->get_logger(), "Action clients not ready yet. Please wait.");
            return;
        }

        // Stop any existing playback
        stop();
        
        // Reset state for new playback
        pending_events_ = std::queue<Event>();  // Clear the event queue
        
        // Schedule all events with absolute times
        schedule_events();
        
        // Start playback
        is_playing_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting to play: %s", current_song_.name.c_str());
        
        // Start the playback loop
        double update_interval = 0.01; // 10ms update interval for precise timing
        playback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(update_interval * 1000)),
            std::bind(&ConfigurableSongPlayer::playback_loop, this)
        );
    }

    /**
     * @brief Stop the currently playing song
     */
    void stop() {
        is_playing_ = false;
        if (playback_timer_) {
            playback_timer_->cancel();
        }
        
        // Reset all solenoids to off state
        reset_all_solenoids();
        
        RCLCPP_INFO(this->get_logger(), "Stopped playing");
    }

private:
    // Action clients for servo controllers
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_servo_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_servo_client_;
    bool left_servo_client_ready_ = false;
    bool right_servo_client_ready_ = false;
    
    // Publishers for solenoid controllers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_solenoid_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_solenoid_publisher_;
    
    // Timer for checking client readiness
    rclcpp::TimerBase::SharedPtr check_clients_timer_;
    
    // Song playback timer
    rclcpp::TimerBase::SharedPtr playback_timer_;
    
    // Song data
    Song current_song_;
    bool is_playing_ = false;
    std::queue<Event> pending_events_;
    
    // Track which solenoids are currently active
    std::vector<bool> left_active_solenoids_ = {false, false, false, false, false};
    std::vector<bool> right_active_solenoids_ = {false, false, false, false, false};
    
    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr song_command_sub_;
    
    // Joint name mapping
    std::vector<std::string> left_finger_servo_joints_;
    std::vector<std::string> right_finger_servo_joints_;
    
    // Finger index mapping (maps finger name to index in the joint arrays)
    std::map<std::string, int> left_finger_index_;
    std::map<std::string, int> right_finger_index_;

    /**
     * @brief Reset all solenoids to off state
     */
    void reset_all_solenoids() {
        // Turn off all left hand solenoids
        auto left_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        left_msg->data.resize(5, 0.0);  // 5 solenoids in the hand
        left_solenoid_publisher_->publish(std::move(left_msg));
        
        // Turn off all right hand solenoids
        auto right_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        right_msg->data.resize(5, 0.0);  // 5 solenoids in the hand
        right_solenoid_publisher_->publish(std::move(right_msg));
        
        // Reset tracking variables
        left_active_solenoids_ = {false, false, false, false, false};
        right_active_solenoids_ = {false, false, false, false, false};
    }

    /**
     * @brief Initialize finger joint name mappings
     */
    void initialize_finger_joint_maps() {
        // Left hand servo joint names (reversed order: 1=pinky, 5=thumb)
        left_finger_servo_joints_ = {
            "lh_servo_1",  // thumb
            "lh_servo_2",  // index
            "lh_servo_3",  // middle
            "lh_servo_4",  // ring
            "lh_servo_5"   // pinky
        };

        // Right hand servo joint names (normal order: 1=thumb, 5=pinky)
        right_finger_servo_joints_ = {
            "rh_servo_1",  // thumb
            "rh_servo_2",  // index
            "rh_servo_3",  // middle
            "rh_servo_4",  // ring
            "rh_servo_5"   // pinky
        };

        // Create mapping from finger name to index
        // For left hand, finger mapping is reversed
        left_finger_index_ = {
            {"thumb", 0},  // pinky maps to lh_servo_1
            {"index", 1},   // ring maps to lh_servo_2
            {"middle", 2}, // middle maps to lh_servo_3
            {"ring", 3},  // index maps to lh_servo_4
            {"pinky", 4}   // thumb maps to lh_servo_5
        };

        // For right hand, finger mapping is normal
        right_finger_index_ = {
            {"thumb", 0},  // thumb maps to rh_servo_1
            {"index", 1},  // index maps to rh_servo_2
            {"middle", 2}, // middle maps to rh_servo_3
            {"ring", 3},   // ring maps to rh_servo_4
            {"pinky", 4}   // pinky maps to rh_servo_5
        };
    }

    /**
     * @brief Check if action clients are ready to send goals
     */
    void check_clients_ready() {
        if (!left_servo_client_ready_) {
            left_servo_client_ready_ = left_servo_client_->action_server_is_ready();
            if (left_servo_client_ready_) {
                RCLCPP_INFO(this->get_logger(), "Left servo controller client is ready");
            }
        }
        
        if (!right_servo_client_ready_) {
            right_servo_client_ready_ = right_servo_client_->action_server_is_ready();
            if (right_servo_client_ready_) {
                RCLCPP_INFO(this->get_logger(), "Right servo controller client is ready");
            }
        }
        
        if (left_servo_client_ready_ && right_servo_client_ready_) {
            check_clients_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "All clients are ready. You can now play a song.");
        }
    }

    /**
     * @brief Schedule all events with absolute times
     * 
     * Converts the delta times in the song events to absolute scheduled times
     * and queues them for playback
     */
    void schedule_events() {
        if (current_song_.events.empty()) {
            return;
        }
        
        // Starting time is now
        auto current_time = this->now();
        double accumulated_time = 0.0;
        
        // Calculate the tempo scaling factor
        // At 120 BPM, quarter note = 0.5 seconds
        double tempo_scaling_factor = 60.0 / current_song_.tempo_bpm;
        
        RCLCPP_INFO(this->get_logger(), "Song tempo: %.1f BPM (scaling factor: %.3f)",
                   current_song_.tempo_bpm, tempo_scaling_factor);
        
        // Schedule each event based on its delta time
        for (auto& event : current_song_.events) {
            // Scale the delta time by the tempo factor
            double scaled_delta = event.delta_time * tempo_scaling_factor;
            
            // Add the scaled delta time to get the absolute time for this event
            accumulated_time += scaled_delta;
            
            // Create a copy of the event for the queue
            Event scheduled_event = event;
            scheduled_event.scheduled_time = current_time + rclcpp::Duration::from_seconds(accumulated_time);
            
            // Queue the event
            pending_events_.push(scheduled_event);
        }
        
        RCLCPP_INFO(this->get_logger(), "Scheduled %zu events", pending_events_.size());
    }

    /**
     * @brief Main playback loop
     * 
     * Checks for events that are due to be processed and executes them
     */
    void playback_loop() {
        if (!is_playing_) {
            if (playback_timer_) {
                playback_timer_->cancel();
            }
            return;
        }
        
        // If no more events, playback is finished
        if (pending_events_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Song finished!");
            stop();
            return;
        }
        
        // Get the current time
        auto current_time = this->now();
        
        // Process all events that are due
        while (!pending_events_.empty()) {
            const Event& next_event = pending_events_.front();
            
            // If this event is scheduled for the future, break and wait
            if (next_event.scheduled_time > current_time) {
                break;
            }
            
            // Process this event now
            process_event(next_event);
            
            // Remove the event from the queue
            pending_events_.pop();
        }
    }

    /**
     * @brief Process a single event
     * 
     * @param event The event to process
     */
    void process_event(const Event& event) {
        RCLCPP_INFO(this->get_logger(), "Processing event: %s %s %s %s %.1f",
                   event.note_name.c_str(), event.finger.c_str(), event.hand.c_str(), 
                   event.component.c_str(), event.position);
        
        // Handle left hand
        if (event.hand == "left") {
            // Check if the finger exists
            if (left_finger_index_.find(event.finger) == left_finger_index_.end()) {
                RCLCPP_WARN(this->get_logger(), "Unknown left hand finger: %s", event.finger.c_str());
                return;
            }
            
            int finger_idx = left_finger_index_[event.finger];
            
            // Process based on component type
            if (event.component == "servo") {
                send_left_servo_command(finger_idx, event.position);
            } else if (event.component == "solenoid") {
                send_left_solenoid_command(finger_idx, event.position);
            }
        }
        // Handle right hand
        else if (event.hand == "right") {
            // Check if the finger exists
            if (right_finger_index_.find(event.finger) == right_finger_index_.end()) {
                RCLCPP_WARN(this->get_logger(), "Unknown right hand finger: %s", event.finger.c_str());
                return;
            }
            
            int finger_idx = right_finger_index_[event.finger];
            
            // Process based on component type
            if (event.component == "servo") {
                send_right_servo_command(finger_idx, event.position);
            } else if (event.component == "solenoid") {
                send_right_solenoid_command(finger_idx, event.position);
            }
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown hand: %s", event.hand.c_str());
        }
    }


    /**
     * @brief Send a command to the left hand servo controller
     * 
     * @param finger_idx The index of the finger servo to control
     * @param angle The angle to set the servo to
     */
    void send_left_servo_command(int finger_idx, double angle) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add the specified finger servo joint
        trajectory.joint_names.push_back(left_finger_servo_joints_[finger_idx]);
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {angle};
        
        // Set time from start (using command_period as in your test)
        point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // 50ms as in your test
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal with result callback
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        // Add goal response callback to check if the goal was accepted
        send_goal_options.goal_response_callback =
            [this, finger_idx](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Left servo %d goal was rejected by server", finger_idx);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Left servo %d goal accepted by server", finger_idx);
                }
            };
        
        // Add result callback to know when the trajectory completed
        send_goal_options.result_callback =
            [this, finger_idx, angle](const GoalHandleFollowJointTrajectory::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_DEBUG(this->get_logger(), "Left servo %d moved to %.2f degrees successfully", 
                            finger_idx, angle);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Left servo %d command failed with code %d", 
                                finger_idx, static_cast<int>(result.code));
                }
            };
        
        left_servo_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Send a command to the right hand servo controller
     * 
     * @param finger_idx The index of the finger servo to control
     * @param angle The angle to set the servo to
     */
    void send_right_servo_command(int finger_idx, double angle) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add the specified finger servo joint
        trajectory.joint_names.push_back(right_finger_servo_joints_[finger_idx]);
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {angle};
        
        // Set time from start (using command_period as in your test)
        point.time_from_start = rclcpp::Duration::from_seconds(0.05);  // 50ms as in your test
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal with result callback
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        // Add goal response callback to check if the goal was accepted
        send_goal_options.goal_response_callback =
            [this, finger_idx](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Right servo %d goal was rejected by server", finger_idx);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Right servo %d goal accepted by server", finger_idx);
                }
            };
        
        // Add result callback to know when the trajectory completed
        send_goal_options.result_callback =
            [this, finger_idx, angle](const GoalHandleFollowJointTrajectory::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_DEBUG(this->get_logger(), "Right servo %d moved to %.2f degrees successfully", 
                            finger_idx, angle);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Right servo %d command failed with code %d", 
                                finger_idx, static_cast<int>(result.code));
                }
            };
        
        right_servo_client_->async_send_goal(goal_msg, send_goal_options);
    }    

    // /**
    //  * @brief Send a command to the left hand servo controller
    //  * 
    //  * @param finger_idx The index of the finger servo to control
    //  * @param angle The angle to set the servo to
    //  */
    // void send_left_servo_command(int finger_idx, double angle) {
    //     auto goal_msg = FollowJointTrajectory::Goal();
        
    //     // Create trajectory
    //     trajectory_msgs::msg::JointTrajectory trajectory;
        
    //     // Add the specified finger servo joint
    //     trajectory.joint_names.push_back(left_finger_servo_joints_[finger_idx]);
        
    //     // Create a trajectory point
    //     trajectory_msgs::msg::JointTrajectoryPoint point;
    //     point.positions = {angle};
        
    //     // Set time from start (use a fixed short duration)
    //     point.time_from_start.sec = 0;
    //     point.time_from_start.nanosec = 0; 
        
    //     trajectory.points.push_back(point);
    //     goal_msg.trajectory = trajectory;
        
    //     // Send the goal
    //     RCLCPP_DEBUG(this->get_logger(), "Sending goal to left servo controller for finger %d", finger_idx);
    //     auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    //     send_goal_options.goal_response_callback =
    //         [this, finger_idx](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
    //             if (!goal_handle) {
    //                 RCLCPP_ERROR(this->get_logger(), "Left servo %d goal was rejected by server", finger_idx);
    //             } else {
    //                 RCLCPP_DEBUG(this->get_logger(), "Left servo %d goal accepted by server", finger_idx);
    //             }
    //         };
        
    //     left_servo_client_->async_send_goal(goal_msg, send_goal_options);
    // }

    // /**
    //  * @brief Send a command to the right hand servo controller
    //  * 
    //  * @param finger_idx The index of the finger servo to control
    //  * @param angle The angle to set the servo to
    //  */
    // void send_right_servo_command(int finger_idx, double angle) {
    //     auto goal_msg = FollowJointTrajectory::Goal();
        
    //     // Create trajectory
    //     trajectory_msgs::msg::JointTrajectory trajectory;
        
    //     // Add the specified finger servo joint
    //     trajectory.joint_names.push_back(right_finger_servo_joints_[finger_idx]);
        
    //     // Create a trajectory point
    //     trajectory_msgs::msg::JointTrajectoryPoint point;
    //     point.positions = {angle};
        
    //     // Set time from start (use a fixed short duration)
    //     point.time_from_start.sec = 0;
    //     point.time_from_start.nanosec = 0; 
        
    //     trajectory.points.push_back(point);
    //     goal_msg.trajectory = trajectory;
        
    //     // Send the goal
    //     RCLCPP_DEBUG(this->get_logger(), "Sending goal to right servo controller for finger %d", finger_idx);
    //     auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    //     send_goal_options.goal_response_callback =
    //         [this, finger_idx](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
    //             if (!goal_handle) {
    //                 RCLCPP_ERROR(this->get_logger(), "Right servo %d goal was rejected by server", finger_idx);
    //             } else {
    //                 RCLCPP_DEBUG(this->get_logger(), "Right servo %d goal accepted by server", finger_idx);
    //             }
    //         };
        
    //     right_servo_client_->async_send_goal(goal_msg, send_goal_options);
    // }

/**
     * @brief Send a command to the left hand solenoid controller
     * 
     * @param finger_idx The index of the finger solenoid to control
     * @param state The state to set the solenoid to (0 or 1)
     */
    void send_left_solenoid_command(int finger_idx, double state) {
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        
        // Initialize array with current states
        msg->data.resize(5, 0.0);
        for (int i = 0; i < 5; i++) {
            if (left_active_solenoids_[i]) {
                msg->data[i] = 1.0;
            }
        }
        
        // Set the state for the specified finger
        msg->data[finger_idx] = state;
        
        // Update tracking state
        left_active_solenoids_[finger_idx] = (state > 0.0);
        
        // Publish the command
        RCLCPP_DEBUG(this->get_logger(), "Sending command to left solenoid controller for finger %d: %f", 
                    finger_idx, state);
        left_solenoid_publisher_->publish(std::move(msg));
    }

    /**
     * @brief Send a command to the right hand solenoid controller
     * 
     * @param finger_idx The index of the finger solenoid to control
     * @param state The state to set the solenoid to (0 or 1)
     */
    void send_right_solenoid_command(int finger_idx, double state) {
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        
        // Initialize array with current states
        msg->data.resize(5, 0.0);
        for (int i = 0; i < 5; i++) {
            if (right_active_solenoids_[i]) {
                msg->data[i] = 1.0;
            }
        }
        
        // Set the state for the specified finger
        msg->data[finger_idx] = state;
        
        // Update tracking state
        right_active_solenoids_[finger_idx] = (state > 0.0);
        
        // Publish the command
        RCLCPP_DEBUG(this->get_logger(), "Sending command to right solenoid controller for finger %d: %f", 
                    finger_idx, state);
        right_solenoid_publisher_->publish(std::move(msg));
    }

    /**
     * @brief Handle song command messages
     * 
     * Expected command format: "command:argument"
     * e.g., "load:song.csv", "play", "stop"
     * 
     * @param msg The command message
     */
    void song_command_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        
        // Split command and argument
        std::string cmd, arg;
        size_t pos = command.find(':');
        if (pos != std::string::npos) {
            cmd = command.substr(0, pos);
            arg = command.substr(pos + 1);
        } else {
            cmd = command;
        }
        
        // Process command
        if (cmd == "load" && !arg.empty()) {
            RCLCPP_INFO(this->get_logger(), "Loading song: %s", arg.c_str());
            bool success = load_song_from_file(arg);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Song '%s' loaded successfully", arg.c_str());
                
                // If we're currently playing, switch to the new song
                if (is_playing_) {
                    RCLCPP_INFO(this->get_logger(), "Switching to newly loaded song");
                    // Stop current playback but don't reset solenoids immediately
                    is_playing_ = false;
                    if (playback_timer_) {
                        playback_timer_->cancel();
                    }
                    
                    // Schedule events for the new song
                    schedule_events();
                    
                    // Start playback of the new song
                    is_playing_ = true;
                    
                    // Start the playback loop with the new song
                    double update_interval = 0.01; // 10ms update interval
                    playback_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<int>(update_interval * 1000)),
                        std::bind(&ConfigurableSongPlayer::playback_loop, this)
                    );
                }
            }
        } else if (cmd == "play") {
            RCLCPP_INFO(this->get_logger(), "Command: Play");
            play();
        } else if (cmd == "stop") {
            RCLCPP_INFO(this->get_logger(), "Command: Stop");
            stop();
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConfigurableSongPlayer>();
    
    // If a song file is specified as a command-line argument, load it
    if (argc > 1) {
        if (node->load_song_from_file(argv[1])) {
            // Wait a moment for connections to establish
            std::this_thread::sleep_for(std::chrono::seconds(2));
            node->play();
        }
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
