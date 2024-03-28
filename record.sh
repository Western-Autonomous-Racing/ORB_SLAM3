while getopts s:t: flag
do
  case "${flag}" in
    s) session_name=${OPTARG};;
    t) topics=${OPTARG};;
  esac
done

# if session is empty provide default name
if [ -z "$session_name" ]
then
  session_name="recording"
fi

# of session is empty provide default topics
if [ -z "$topics" ]
then
  topics="/raw_map_points /egocar/odom /refined_map_points"
fi

splitsize=10000000000
destination="/home/$(whoami)/Projects/Data/processed_cloudpoints/"
session_name="$(date '+%Y-%m-%d-T%H-%M-%S')-$session_name"

recording_path="$destination/$session_name"

if [ ! -d "$recording_path" ]; then
  mkdir -p "$recording_path"
fi
echo "Recording to $recording_path"

cd ~/Projects/orbslam3_ws
source install/setup.bash
ros2 run orbslam3_ros2 map stereo src/ORB_SLAM3/Vocabulary/ORBvoc.txt src/ORB_SLAM3/configuration/D430.yaml false true &

recording_file="$recording_path/recording"
FS=' ' read -r -a topic_array <<< "$topics"

# ros2 bag record -o "$recording_file" -b $splitsize "${topic_array[@]}" &
ros2 bag record -o "$recording_file" "${topic_array[@]}" &
# ros2 bag record -a -o "$recording_file" &
record_pid=$!

# Add a trap to stop recording when the script is terminated
trap 'kill $record_pid' SIGINT SIGTERM

# Ignore SIGINT and SIGTERM in the wait command
trap '' SIGINT SIGTERM

# Wait for the script to be terminated
wait $record_pid

# Reset the trap to the default behavior
trap - SIGINT SIGTERM
