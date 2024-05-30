tmux new -d -s whisper

tmux split-window -v -t whisper:0
tmux split-window -v -t whisper:0.1

tmux send-keys -t whisper:0.0 C-z 'rosrun whisperwrapper whisperwrapper.py' Enter
tmux send-keys -t whisper:0.1 C-z 'rostopic echo /stt/listening' Enter
tmux send-keys -t whisper:0.2 C-z 'rostopic echo /planner_intention' Enter