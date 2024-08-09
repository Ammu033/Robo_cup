tmux new -d -s functioncalling
tmux split-window -v -t functioncalling:0
tmux split-window -h -t functioncalling:0.0
tmux split-window -h -t functioncalling:0.2
tmux split-window -v -t functioncalling:0.2
tmux split-window -v -t functioncalling:0.4

tmux send-keys -t functioncalling:0.0 C-z 'rosrun ollamawrapper ollamawrapper.py' Enter
tmux send-keys -t functioncalling:0.1 C-z 'echo -e "\n\n\nExample CLI service call:\nrosservice call /stt/ollamacall \"input: my name is alice\"\n"' Enter Enter
tmux send-keys -t functioncalling:0.4 C-z 'echo -e "\n\n\nExample to set intent:\nrostopic pub /planner_intention std_msgs/String \"guest_name\" -1\n"' Enter Enter
tmux send-keys -t functioncalling:0.3 C-z 'rostopic echo /ollama_response' Enter
tmux send-keys -t functioncalling:0.5 C-z 'rostopic echo /stt/listening' Enter
tmux send-keys -t functioncalling:0.2 C-z 'rostopic echo /ollama_output' Enter

tmux select-pane -t functioncalling:0.1