#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

/**
 * @brief Chord struct to represent multiple notes played simultaneously
 */
struct Chord {
    std::string name;                                  // Chord name (e.g., C-Major)
    std::string hand;                                  // Hand to use (left, right, both)
    std::vector<std::string> fingers;                  // List of fingers to use
    std::vector<double> servo_positions;               // Servo positions for each finger
    std::vector<double> solenoid_activations;          // Solenoid activations for each finger
    double duration;                                   // Chord duration in beats
};

/**
 * @brief ChordSequence struct to represent a sequence of chords
 */
struct ChordSequence {
    std::string name;                                  // Sequence name
    double tempo_bpm;                                  // Tempo in beats per minute
    std::vector<Chord> chords;                         // All chords in the sequence
};

/**
 * @brief ChordPlayer class
 * 
 * A ROS2 node that sends action goals to control multiple fingers
 * simultaneously for playing chords
 */
class ChordPlayer : public rclcpp::Node {
public:
    ChordPlayer() : Node("chord_player") {
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
            std::bind(&ChordPlayer::check_clients_ready, this)
        );

        // Define finger joint mappings
        initialize_joint_maps();

        RCLCPP_INFO(this->get_logger(), "ChordPlayer node initialized");
    }

    /**
     * @brief Set the chord sequence to play
     * 
     * @param sequence The chord sequence to play
     */
    void set_sequence(const ChordSequence& sequence) {
        sequence_ = sequence;
        RCLCPP_INFO(this->get_logger(), "Chord sequence set: %s (Tempo: %.1f BPM)", 
                   sequence.name.c_str(), sequence.tempo_bpm);
    }

    /**
     * @brief Start playing the chord sequence
     */
    void play() {
        if (sequence_.chords.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No chord sequence loaded. Please set a sequence first.");
            return;
        }

        if (!left_hand_client_ready_ || !right_hand_client_ready_) {
            RCLCPP_ERROR(this->get_logger(), "Action clients not ready yet. Please wait.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting to play: %s", sequence_.name.c_str());
        current_chord_index_ = 0;
        play_next_chord();
    }

private:
    // Action clients
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_hand_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_hand_client_;
    bool left_hand_client_ready_ = false;
    bool right_hand_client_ready_ = false;
    rclcpp::TimerBase::SharedPtr check_clients_timer_;
    
    // Chord sequence data
    ChordSequence sequence_;
    size_t current_chord_index_ = 0;
    
    // Joint mapping
    std::map<std::string, std::string> left_finger_servo_joints_;
    std::map<std::string, std::string> left_finger_solenoid_joints_;
    std::map<std::string, std::string> right_finger_servo_joints_;
    std::map<std::string, std::string> right_finger_solenoid_joints_;

    /**
     * @brief Initialize joint name mappings for fingers
     */
    void initialize_joint_maps() {
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
            RCLCPP_INFO(this->get_logger(), "Both hand clients are ready. You can now play a chord sequence.");
        }
    }

    /**
     * @brief Play the next chord in the sequence
     */
    void play_next_chord() {
        if (current_chord_index_ >= sequence_.chords.size()) {
            RCLCPP_INFO(this->get_logger(), "Chord sequence finished!");
            return;
        }

        const Chord& chord = sequence_.chords[current_chord_index_];
        RCLCPP_INFO(this->get_logger(), "Playing chord: %s (Hand: %s, Duration: %.1f beats)",
                   chord.name.c_str(), chord.hand.c_str(), chord.duration);

        // Process the chord based on which hand to use
        if (chord.hand == "left") {
            send_left_hand_chord(chord);
        } else if (chord.hand == "right") {
            send_right_hand_chord(chord);
        } else if (chord.hand == "both") {
            send_left_hand_chord(chord);
            send_right_hand_chord(chord);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown hand: %s", chord.hand.c_str());
        }

        // Schedule the next chord based on the duration
        double beat_duration_sec = 60.0 / sequence_.tempo_bpm;
        double chord_duration_sec = chord.duration * beat_duration_sec;
        
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(chord_duration_sec * 1000)),
            [this]() {
                current_chord_index_++;
                play_next_chord();
                return true;  // One-shot timer
            }
        );
    }

    /**
     * @brief Send a chord to the left hand controller
     * 
     * @param chord The chord to play
     */
    void send_left_hand_chord(const Chord& chord) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add all the finger servo and solenoid joints
        for (size_t i = 0; i < chord.fingers.size(); i++) {
            const std::string& finger = chord.fingers[i];
            
            // Check if the finger mapping exists
            if (left_finger_servo_joints_.find(finger) == left_finger_servo_joints_.end() ||
                left_finger_solenoid_joints_.find(finger) == left_finger_solenoid_joints_.end()) {
                RCLCPP_WARN(this->get_logger(), "Unknown left finger: %s", finger.c_str());
                continue;
            }
            
            trajectory.joint_names.push_back(left_finger_servo_joints_[finger]);
            trajectory.joint_names.push_back(left_finger_solenoid_joints_[finger]);
        }
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Add all position values
        for (size_t i = 0; i < chord.fingers.size(); i++) {
            if (i < chord.servo_positions.size() && i < chord.solenoid_activations.size()) {
                point.positions.push_back(chord.servo_positions[i]);
                point.positions.push_back(chord.solenoid_activations[i]);
            }
        }
        
        // Set time from start
        double beat_duration_sec = 60.0 / sequence_.tempo_bpm;
        point.time_from_start.sec = static_cast<int>(beat_duration_sec);
        point.time_from_start.nanosec = 
            static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending left hand chord with %zu fingers", chord.fingers.size());
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Left hand chord goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Left hand chord goal accepted by server");
                }
            };
        
        left_hand_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Send a chord to the right hand controller
     * 
     * @param chord The chord to play
     */
    void send_right_hand_chord(const Chord& chord) {
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create trajectory
        trajectory_msgs::msg::JointTrajectory trajectory;
        
        // Add all the finger servo and solenoid joints
        for (size_t i = 0; i < chord.fingers.size(); i++) {
            const std::string& finger = chord.fingers[i];
            
            // Check if the finger mapping exists
            if (right_finger_servo_joints_.find(finger) == right_finger_servo_joints_.end() ||
                right_finger_solenoid_joints_.find(finger) == right_finger_solenoid_joints_.end()) {
                RCLCPP_WARN(this->get_logger(), "Unknown right finger: %s", finger.c_str());
                continue;
            }
            
            trajectory.joint_names.push_back(right_finger_servo_joints_[finger]);
            trajectory.joint_names.push_back(right_finger_solenoid_joints_[finger]);
        }
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Add all position values
        for (size_t i = 0; i < chord.fingers.size(); i++) {
            if (i < chord.servo_positions.size() && i < chord.solenoid_activations.size()) {
                point.positions.push_back(chord.servo_positions[i]);
                point.positions.push_back(chord.solenoid_activations[i]);
            }
        }
        
        // Set time from start
        double beat_duration_sec = 60.0 / sequence_.tempo_bpm;
        point.time_from_start.sec = static_cast<int>(beat_duration_sec);
        point.time_from_start.nanosec = 
            static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending right hand chord with %zu fingers", chord.fingers.size());
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Right hand chord goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Right hand chord goal accepted by server");
                }
            };
        
        right_hand_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

/**
 * @brief Create an example chord sequence
 * 
 * @return ChordSequence 
 */
ChordSequence create_example_chord_sequence() {
    ChordSequence sequence;
    sequence.name = "Simple Chord Progression";
    sequence.tempo_bpm = 60.0;  // Slow tempo for better visibility
    
    // C Major chord (C-E-G) with right hand
    Chord c_major;
    c_major.name = "C Major";
    c_major.hand = "right";
    c_major.fingers = {"thumb", "middle", "pinky"};          // C-E-G fingering
    c_major.servo_positions = {30.0, 45.0, 60.0};           // Servo positions for each finger
    c_major.solenoid_activations = {1.0, 1.0, 1.0};         // Activate all solenoids
    c_major.duration = 2.0;                                 // Hold for 2 beats
    
    // G Major chord (G-B-D) with left hand
    Chord g_major;
    g_major.name = "G Major";
    g_major.hand = "left";
    g_major.fingers = {"thumb", "index", "middle"};          // G-B-D fingering
    g_major.servo_positions = {60.0, 50.0, 40.0};           // Servo positions for each finger
    g_major.solenoid_activations = {1.0, 1.0, 1.0};         // Activate all solenoids
    g_major.duration = 2.0;                                 // Hold for 2 beats
    
    // A Minor chord (A-C-E) with both hands
    Chord a_minor;
    a_minor.name = "A Minor";
    a_minor.hand = "both";                                  // Use both hands for this chord
    a_minor.fingers = {"index", "middle"};                  // Use index and middle on both hands
    a_minor.servo_positions = {45.0, 55.0};                // Servo positions for each finger
    a_minor.solenoid_activations = {1.0, 1.0};             // Activate both solenoids
    a_minor.duration = 2.0;                                // Hold for 2 beats
    
    // F Major chord (F-A-C) with right hand
    Chord f_major;
    f_major.name = "F Major";
    f_major.hand = "right";
    f_major.fingers = {"thumb", "middle", "pinky"};          // F-A-C fingering
    f_major.servo_positions = {40.0, 50.0, 35.0};           // Servo positions for each finger
    f_major.solenoid_activations = {1.0, 1.0, 1.0};         // Activate all solenoids
    f_major.duration = 2.0;                                 // Hold for 2 beats
    
    // Add chords to sequence
    sequence.chords.push_back(c_major);
    sequence.chords.push_back(g_major);
    sequence.chords.push_back(a_minor);
    sequence.chords.push_back(f_major);
    
    return sequence;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChordPlayer>();
    
    // Create and set a chord sequence
    ChordSequence sequence = create_example_chord_sequence();
    node->set_sequence(sequence);
    
    // Wait a moment for connections to establish
    std::this_thread::sleep_for(std::chrono::seconds(2));
    node->play();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}