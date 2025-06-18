
#!/bin/bash

# Node name as the first argument
NODE_NAME=$1

if [ -z "$NODE_NAME" ]; then
  echo "Usage: $0 <node_name>"
  exit 1
fi

# Get the list of all topics
TOPICS=$(rostopic list)

# Filter topics by the node name
NODE_TOPICS=""
for TOPIC in $TOPICS; do
  PUBLISHERS=$(rostopic info $TOPIC | grep -oP "(?<=Publishers: ).*")
  if echo $PUBLISHERS | grep -q $NODE_NAME; then
    NODE_TOPICS="$NODE_TOPICS $TOPIC"
  fi
done

if [ -z "$NODE_TOPICS" ]; then
  echo "No topics found for node $NODE_NAME"
  exit 1
fi

# Start recording the topics into a rosbag
rosbag record -O ${NODE_NAME//\//_}.bag $NODE_TOPICS