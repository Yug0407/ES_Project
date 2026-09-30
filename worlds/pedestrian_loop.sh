#!/bin/bash
# Makes pedestrian_1 and pedestrian_2 slide back and forth forever.
# Run this in a SEPARATE terminal, AFTER Gazebo is already open and running (unpaused).

echo "Starting pedestrian loop... press Ctrl+C to stop."

DIRECTION=1

while true; do
  if [ "$DIRECTION" -eq 1 ]; then
    gz topic -t "/model/pedestrian_1/joint/slide_joint/cmd_vel" -m gz.msgs.Double -p "data: 0.4"
    gz topic -t "/model/pedestrian_2/joint/slide_joint/cmd_vel" -m gz.msgs.Double -p "data: -0.3"
    DIRECTION=0
  else
    gz topic -t "/model/pedestrian_1/joint/slide_joint/cmd_vel" -m gz.msgs.Double -p "data: -0.4"
    gz topic -t "/model/pedestrian_2/joint/slide_joint/cmd_vel" -m gz.msgs.Double -p "data: 0.3"
    DIRECTION=1
  fi
  sleep 5
done
