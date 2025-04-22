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
 * @brief Note struct to represent a musical note to be played
 * 
 * Holds the finger position and activation information for a single note
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
 * 
 * Contains all the notes of a song, along with tempo information
 */
struct Song {
    std::string name;                  // Song name
    double tempo_bpm;                  // Tempo in beats per minute
    std::vector<Note> notes;           // All notes in the song
};

/**
 * @brief SongPlayer class
 * 
 * A ROS2 node that sends action goals to control the fingers
 * for playing a song on the piano
 */
class SongPlayer : public rclcpp::Node {
public:
    SongPlayer() : Node("song_player") {
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
            std::bind(&SongPlayer::check_clients_ready, this)
        );

        // Define the map of finger names to joint names
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

        RCLCPP_INFO(this->get_logger(), "SongPlayer node initialized");
    }

    /**
     * @brief Set the song to play
     * 
     * @param song The song to play
     */
    void set_song(const Song& song) {
        song_ = song;
        RCLCPP_INFO(this->get_logger(), "Song set: %s (Tempo: %.1f BPM)", 
                   song.name.c_str(), song.tempo_bpm);
    }

    /**
     * @brief Start playing the song
     */
    void play() {
        if (song_.notes.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No song loaded. Please set a song first.");
            return;
        }

        if (!left_hand_client_ready_ || !right_hand_client_ready_) {
            RCLCPP_ERROR(this->get_logger(), "Action clients not ready yet. Please wait.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting to play: %s", song_.name.c_str());
        current_note_index_ = 0;
        play_next_note();
    }

private:
    // Action clients
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_hand_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_hand_client_;
    bool left_hand_client_ready_ = false;
    bool right_hand_client_ready_ = false;
    rclcpp::TimerBase::SharedPtr check_clients_timer_;
    
    // Song data
    Song song_;
    size_t current_note_index_ = 0;
    
    // Joint mapping
    std::map<std::string, std::string> left_finger_servo_joints_;
    std::map<std::string, std::string> left_finger_solenoid_joints_;
    std::map<std::string, std::string> right_finger_servo_joints_;
    std::map<std::string, std::string> right_finger_solenoid_joints_;

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
        if (current_note_index_ >= song_.notes.size()) {
            RCLCPP_INFO(this->get_logger(), "Song finished!");
            return;
        }

        const Note& note = song_.notes[current_note_index_];
        RCLCPP_INFO(this->get_logger(), "Playing note: %s (Finger: %s, Hand: %s)",
                   note.name.c_str(), note.finger.c_str(), note.hand.c_str());

        // Determine which hand to use
        if (note.hand == "left") {
            send_left_hand_goal(note);
        } else {
            send_right_hand_goal(note);
        }

        // Schedule the next note based on the duration
        double beat_duration_sec = 60.0 / song_.tempo_bpm;
        double note_duration_sec = note.duration * beat_duration_sec;
        
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(note_duration_sec * 1000)),
            [this]() {
                current_note_index_++;
                play_next_note();
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
        trajectory.joint_names.push_back(left_finger_servo_joints_[note.finger]);
        trajectory.joint_names.push_back(left_finger_solenoid_joints_[note.finger]);
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {note.servo_position, note.solenoid_activation};
        
        // Set time from start
        double beat_duration_sec = 60.0 / song_.tempo_bpm;
        point.time_from_start.sec = static_cast<int>(beat_duration_sec);
        point.time_from_start.nanosec = 
            static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal to left hand controller");
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Left hand goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Left hand goal accepted by server");
                }
            };
        
        left_hand_client_->async_send_goal(goal_msg, send_goal_options);
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
        trajectory.joint_names.push_back(right_finger_servo_joints_[note.finger]);
        trajectory.joint_names.push_back(right_finger_solenoid_joints_[note.finger]);
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {note.servo_position, note.solenoid_activation};
        
        // Set time from start
        double beat_duration_sec = 60.0 / song_.tempo_bpm;
        point.time_from_start.sec = static_cast<int>(beat_duration_sec);
        point.time_from_start.nanosec = 
            static_cast<uint32_t>((beat_duration_sec - static_cast<int>(beat_duration_sec)) * 1e9);
        
        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;
        
        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal to right hand controller");
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Right hand goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Right hand goal accepted by server");
                }
            };
        
        right_hand_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

/**
 * @brief Create a song from predefined notes and timing
 * 
 * This example creates a simple "Mary Had a Little Lamb" melody
 * 
 * @return Song 
 */
Song create_mary_had_a_little_lamb() {
    Song song;
    song.name = "Mary Had a Little Lamb";
    song.tempo_bpm = 100.0;  // Moderate tempo
    
    // Define the notes
    // Mary had a little lamb: E D C D E E E (rest) D D D (rest) E G G (rest)
    // E D C D E E E E D D E D C
    
    // For simplicity, let's map notes to specific fingers and positions
    // C = left pinky, D = left ring, E = left middle, G = left index
    
    // First phrase: E D C D E E E (rest)
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 1.0});   // C
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 2.0});  // E (hold for 2 beats)
    
    // Second phrase: D D D (rest) E G G (rest)
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 2.0});    // D (hold for 2 beats)
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"G4", "index", "left", 45.0, 1.0, 1.0});   // G
    song.notes.push_back({"G4", "index", "left", 45.0, 1.0, 2.0});   // G (hold for 2 beats)
    
    // Third phrase: E D C D E E E E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 1.0});   // C
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    
    // Final phrase: D D E D C
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});  // E
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});    // D
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 2.0});   // C (hold for 2 beats)
    
    return song;
}

/**
 * @brief Create a song for "Twinkle Twinkle Little Star"
 * 
 * @return Song 
 */
Song create_twinkle_twinkle() {
    Song song;
    song.name = "Twinkle Twinkle Little Star";
    song.tempo_bpm = 90.0;  // Slower tempo
    
    // Define the notes
    // Twinkle Twinkle: C C G G A A G (rest) F F E E D D C (rest)
    
    // For this example, we'll use both hands
    // C = left pinky, D = left ring, E = left middle, F = left index
    // G = right index, A = right middle
    
    // First phrase: C C G G A A G (rest)
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 1.0});     // C
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 1.0});     // C
    song.notes.push_back({"G4", "index", "right", 45.0, 1.0, 1.0});    // G
    song.notes.push_back({"G4", "index", "right", 45.0, 1.0, 1.0});    // G
    song.notes.push_back({"A4", "middle", "right", 45.0, 1.0, 1.0});   // A
    song.notes.push_back({"A4", "middle", "right", 45.0, 1.0, 1.0});   // A
    song.notes.push_back({"G4", "index", "right", 45.0, 1.0, 2.0});    // G (hold for 2 beats)
    
    // Second phrase: F F E E D D C (rest)
    song.notes.push_back({"F4", "index", "left", 45.0, 1.0, 1.0});     // F
    song.notes.push_back({"F4", "index", "left", 45.0, 1.0, 1.0});     // F
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});    // E
    song.notes.push_back({"E4", "middle", "left", 45.0, 1.0, 1.0});    // E
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});      // D
    song.notes.push_back({"D4", "ring", "left", 45.0, 1.0, 1.0});      // D
    song.notes.push_back({"C4", "pinky", "left", 45.0, 1.0, 2.0});     // C (hold for 2 beats)
    
    return song;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SongPlayer>();
    
    // Choose which song to play
    Song song;
    if (argc > 1 && std::string(argv[1]) == "twinkle") {
        song = create_twinkle_twinkle();
    } else {
        song = create_mary_had_a_little_lamb();
    }
    
    node->set_song(song);
    
    // Wait a moment for connections to establish
    std::this_thread::sleep_for(std::chrono::seconds(2));
    node->play();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}