#!/bin/bash

# Argument check
if [ "$#" -ne 2 ]; then
  echo -e "\e[31m[ERROR] Incorrect syntax.\e[0m"
  echo "Correct usage: ./remove_collector.sh <world> <name>"
  echo "Example:       ./remove_collector.sh my_world obstacle_2"
  exit 1
fi

WORLD="$1"
NAME="$2"

# Command to remove the entity in Gazebo (type: 2 refers to a 'MODEL')
gz service -s "/world/$WORLD/remove" \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: \"$NAME\", type: 2"

# Check the exit code of the Gazebo command
if [ $? -eq 0 ]; then
  # If everything went well, exit with success. 
  # Python will catch this exit 0 and know it worked.
  exit 0
else
  # If it fails, send the error to the standard error output (>&2)
  echo -e "\e[31m[ERROR] Internal Gazebo failure when trying to remove '$NAME'.\e[0m" >&2
  exit 1
fi
