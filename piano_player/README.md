# Piano Player

This package provides ROS2 nodes to control Bot-Hoven, the piano-playing robot. It includes tools to play predefined songs, chord progressions, or even custom musical pieces from CSV files.

## Components

### Song Player

A simple player that plays a hard-coded song like "Mary Had a Little Lamb" or "Twinkle Twinkle Little Star".

### Chord Player

A player that demonstrates chord progressions by activating multiple fingers simultaneously.

### Configurable Song Player

A flexible player that can load and play songs defined in CSV files. It accepts commands via ROS2 topics and can be controlled programmatically.

## Song File Format

Song files are CSV files structured as follows:

```
name=Song Name,tempo=120
# Format: note_name,finger,hand,servo_position,solenoid_activation,duration
C4,pinky,left,45.0,1.0,1.0
E4,middle,left,45.0,1.0,1.0
G4,index,left,45.0,1.0,2.0
```

Each line in the file (except for comments and metadata) defines a note with these fields:
- `note_name`: Name of the note (e.g., C4, G#3)
- `finger`: Which finger to use (thumb, index, middle, ring, pinky)
- `hand`: Which hand to use (left, right)
- `servo_position`: Servo angle position (in degrees)
- `solenoid_activation`: Solenoid activation state (0=off, 1=on)
- `duration`: Duration in beats (1.0 = quarter note at the given tempo)

The first line can contain metadata:
- `name=`: Song name
- `tempo=`: Tempo in beats per minute (BPM)

## Usage

### Building the package

```bash
cd /bot-hoven-ros
colcon build --packages-select piano_player
source install/setup.bash
```

### Playing a pre-defined song

```bash
ros2 run piano_player song_player
```

### Playing a chord progression

```bash
ros2 run piano_player chord_player
```

### Playing a song from a CSV file

```bash
ros2 run piano_player configurable_song_player path/to/your/song.csv
```

### Using the launch file

The launch file starts both the hardware nodes and the song player:

```bash
ros2 launch piano_player piano_player.launch.py song_file:=mary_had_a_little_lamb.csv
```

#### Launch file parameters:

- `songs_dir`: Directory containing song files
- `song_file`: Song file to play
- `use_real_hardware`: Use real hardware instead of mock hardware

### Sending commands to the configurable player

Commands can be sent via the `song_command` topic:

```bash
# Load a song
ros2 topic pub --once /song_command std_msgs/msg/String "data: 'load:jingle_bells.csv'"

# Start playing
ros2 topic pub --once /song_command std_msgs/msg/String "data: 'play'"

# Stop playing
ros2 topic pub --once /song_command std_msgs/msg/String "data: 'stop'"
```

## Creating Your Own Songs

1. Create a new CSV file following the format described above
2. Place it in the `songs` directory or specify a different directory with the `songs_dir` parameter
3. Run the configurable song player with your file

## Example Songs

Several example songs are provided:
- `mary_had_a_little_lamb.csv`
- `twinkle_twinkle.csv`
- `jingle_bells.csv`

## Customizing for Your Hardware

If your robot has different joint names or requires different parameters, you'll need to modify:

1. The finger-to-joint mappings in the player nodes
2. Servo and solenoid positions in the song files to match your robot's capabilities

## Troubleshooting

- **Action client connections**: If you see errors about action clients not being ready, make sure the hardware controllers are running and have the correct namespaces.
- **Joint limits**: Ensure your servo positions in song files are within the limits defined in your hardware.
- **Timing issues**: Adjust the `tempo_bpm` value if notes are playing too quickly or too slowly.
- **I2C/SPI errors**: If using real hardware and encountering bus errors, check your hardware connections and permissions.