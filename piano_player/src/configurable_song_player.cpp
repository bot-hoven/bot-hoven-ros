#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

/**
 * @brief Note struct to represent a musical note to be played
 */
struct Note {
    std::string name;                  // Note name (e.g., C4, G3)
    std::string finger;                // Finger to use (thumb, index, middle, ring, pinky)
    std::string hand;                  // Hand to use (left, right)
    double servo_position;             // Servo position (angle)
    double solenoid_activation;        // Solenoid activation (0 or 1)
    double duration;                   // Note duration in beats
};

/**
 * @brief Song struct to represent a full musical piece
 */
struct Song {
    std::string name;                  // Song name
    double tempo_bpm;                  // Tempo in beats per minute
    std::vector<Note> notes;           // All notes in the song
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
        
        // Initialize action clients for both hand controllers
        left_hand_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/left_hand_controller/follow_joint_trajectory"
        );

        right_hand_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/right_hand_controller/follow_joint_trajectory"
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

        RCLCPP_INFO(this->get_logger(), "ConfigurableSongPlayer initialized");
    }

    /**
     * @brief Load a song from a CSV file
     * 
     * Expected CSV format:
     * name,finger,hand,servo_position,solenoid_activation,duration
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
            new_song.tempo_bpm = 100.0;  // Default tempo
        }
        
        // Process notes
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
            
            // Expect at least 6 columns
            if (tokens.size() < 6) {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
                continue;
            }
            
            Note note;
            note.name = tokens[0];
            note.finger = tokens[1];
            note.hand = tokens[2];
            
            try {
                note.servo_position = std::stod(tokens[3]);
                note.solenoid_activation = std::stod(tokens[4]);
                note.duration = std::stod(tokens[5]);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Invalid numeric value in line: %s", line.c_str());
                continue;
            }
            
            new_song.notes.push_back(note);
        }
        
        if (new_song.notes.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid notes found in song file: %s", filepath.c_str());
            return false;
        }
        
        // Store the loaded song
        current_song_ = new_song;
        RCLCPP_INFO(this->get_logger(), "Successfully loaded song: %s (Tempo: %.1f BPM, Notes: %zu)",
                   current_song_.name.c_str(), current_song_.tempo_bpm, current_song_.notes.size());
        
        return true;
    }

    /**
     * @brief Play the currently loaded song
     */
    void play() {
        if (current_song_.notes.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No song loaded. Please load a song first.");
            return;
        }

        if (!left_hand_client_ready_ || !right_hand_client_ready_) {
            RCLCPP_ERROR(this->get_logger(), "Action clients not ready yet. Please wait.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting to play: %s", current_song_.name.c_str());
        current_note_index_ = 0;
        is_playing_ = true;
        play_next_note();
    }

    /**
     * @brief Stop the currently playing song
     */
    void stop() {
        is_playing_ = false;
        RCLCPP_INFO(this->get_logger(), "Stopped playing");
    }

private:
    // Action clients
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_hand_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_hand_client_;
    bool left_hand_client_ready_ = false;
    bool right_hand_client_ready_ = false;
    rclcpp::TimerBase::SharedPtr check_clients_timer_;
    
    // Song data
    Song current_song_;
    size_t current_note_index_ = 0;
    bool is_playing_ = false;
    
    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr song_command_sub_;
    
    // Joint mapping
    std::map<std::string, std::string> left_finger_servo_joints_;
    std::map<std::string, std::string> left_finger_solenoid_joints_;
    std::map<std::string, std::string> right_finger_servo_joints_;
    std::map<std::string, std::string> right_finger_solenoid_joints_;

    /**
     * @brief Initialize finger joint name mappings
     */
    void initialize_finger_joint_maps() {
        // Left hand finger joint mappings
        left_finger_servo_joints_ = {
            {"thumb", "left_hand_thumb_servo_joint"},
            {"index", "left_hand_index_finger_servo_joint"},
            {"middle", "left_hand_middle_finger_servo_joint"},
            {"ring", "left_hand_ring_finger_servo_joint"},
            {"pinky", "left_hand_pinky_servo_joint"}
        };

        left_finger_solenoid_joints_ = {
            {"thumb", "left_hand_thumb_solenoid_joint"},
            {"index", "left_hand_index_finger_solenoid_joint"},
            {"middle", "left_hand_middle_finger_solenoid_joint"},
            {"ring", "left_hand_ring_finger_solenoid_joint"},
            {"pinky", "left_hand_pinky_solenoid_joint"}
        };

        // Right hand finger joint mappings
        right_finger_servo_joints_ = {
            {"thumb", "right_hand_thumb_servo_joint"},
            {"index", "right_hand_index_finger_servo_joint"},
            {"middle", "right_hand_middle_finger_servo_joint"},
            {"ring", "right_hand_ring_finger_servo_joint"},
            {"pinky", "right_hand_pinky_servo_joint"}
        };

        right_finger_solenoid_joints_ = {
            {"thumb", "right_hand_thumb_solenoid_joint"},
            {"index", "right_hand_index_finger_solenoid_joint"},
            {"middle", "right_hand_middle_finger_solenoid_joint"},
            {"ring", "right_hand_ring_finger_solenoid_joint"},
            {"pinky", "right_hand_pinky_solenoid_joint"}
        };
    }

    /**
     * @brief Check if action clients are ready to send goals
     */
    void check_clients_ready() {
        if (!left_hand_client_ready_) {
            left_hand_client_ready_ = left_hand_client_->action_server_is_ready();
            if (left_hand_client_ready_) {
                RCLCPP_INFO(this->get_logger(), "Left hand client is ready");
            }
        }
        
        if (!right_hand_client_ready_) {
            right_hand_client_ready_ = right_hand_client_->action_server_is_ready();
            if (right_hand_client_ready_) {
                RCLCPP_INFO(this->get_logger(), "Right hand client is ready");
            }
        }
        
        if (left_hand_client_ready_ && right_hand_client_ready_) {
            check_clients_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Both hand clients are ready. You can now play a song.");
        }
    }

    /**
     * @brief Play the next note in the song
     */
    void play_next_note() {
        if (!is_playing_ || current_note_index_ >= current_song_.notes.size()) {
            if (is_playing_) {
                RCLCPP_INFO(this->get_logger(), "Song finished!");
                is_playing_ = false;
            }
            return;
        }

        const Note& note = current_song_.notes[current_note_index_];
        RCLCPP_INFO(this->get_logger(), "Playing note %zu/%zu: %s (Finger: %s, Hand: %s, Duration: %.1f)",
                   current_note_index_ + 1, current_song_.notes.size(),
                   note.name.c_str(), note.finger.c_str(), note.hand.c_str(), note.duration);

        // Determine which hand to use
        if (note.hand == "left") {
            send_left_hand_goal(note);
        } else if (note.hand == "right") {
            send_right_hand_goal(note);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown hand: %s", note.hand.c_str());
        }

        // Schedule the next note based on the duration
        double beat_duration_sec = 60.0 / current_song_.tempo_bpm;
        double note_duration_sec = note.duration * beat_duration_sec;
        
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(note_duration_sec * 1000)),
            [this]() {
                if (is_playing_) {
                    current_note_index_++;
                    play_next_note();
                }
                // One-shot timer
                return true;
            }
        );
    }

    /**
     * @brief Send a goal to the left hand controller
     * 
     * @param note The note to play
     */
    void send_left_hand_goal(const Note& note) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add the finger servo and solenoid joints
        if (left_finger_servo_joints_.find(note.finger) != left_finger_servo_joints_.end() &&
            left_finger_solenoid_joints_.find(note.finger) != left_finger_solenoid_joints_.end()) {
            
            trajectory.joint_names.push_back(left_finger_servo_joints_[note.finger]);
            trajectory.joint_names.push_back(left_finger_solenoid_joints_[note.finger]);
            
            // Create a trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {note.servo_position, note.solenoid_activation};
            
            // Set time from start
            double beat_duration_sec = 60.0 / current_song_.tempo_bpm;
            point.time_from_start.sec = static_cast<int>(beat_duration_sec);
            point.time_from_start.nanosec = 
                static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
            
            trajectory.points.push_back(point);
            goal_msg.trajectory = trajectory;
            
            // Send the goal
            RCLCPP_DEBUG(this->get_logger(), "Sending goal to left hand controller");
            auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "Left hand goal was rejected by server");
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "Left hand goal accepted by server");
                    }
                };
            
            left_hand_client_->async_send_goal(goal_msg, send_goal_options);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown left hand finger: %s", note.finger.c_str());
        }
    }

    /**
     * @brief Send a goal to the right hand controller
     * 
     * @param note The note to play
     */
    void send_right_hand_goal(const Note& note) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add the finger servo and solenoid joints
        if (right_finger_servo_joints_.find(note.finger) != right_finger_servo_joints_.end() &&
            right_finger_solenoid_joints_.find(note.finger) != right_finger_solenoid_joints_.end()) {
            
            trajectory.joint_names.push_back(right_finger_servo_joints_[note.finger]);
            trajectory.joint_names.push_back(right_finger_solenoid_joints_[note.finger]);
            
            // Create a trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {note.servo_position, note.solenoid_activation};
            
            // Set time from start
            double beat_duration_sec = 60.0 / current_song_.tempo_bpm;
            point.time_from_start.sec = static_cast<int>(beat_duration_sec);
            point.time_from_start.nanosec = 
                static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
            
            trajectory.points.push_back(point);
            goal_msg.trajectory = trajectory;
            
            // Send the goal
            RCLCPP_DEBUG(this->get_logger(), "Sending goal to right hand controller");
            auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "Right hand goal was rejected by server");
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "Right hand goal accepted by server");
                    }
                };
            
            right_hand_client_->async_send_goal(goal_msg, send_goal_options);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown right hand finger: %s", note.finger.c_str());
        }
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
            if (load_song_from_file(arg)) {
                RCLCPP_INFO(this->get_logger(), "Song '%s' loaded successfully", arg.c_str());
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