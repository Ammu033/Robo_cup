tmux new -d -s ollama
tmux split-window -v -t ollama:0
tmux split-window -h -t ollama:0.0
tmux split-window -h -t ollama:0.2
tmux split-window -v -t ollama:0.2
tmux split-window -v -t ollama:0.4

tmux send-keys -t ollama:0.0 C-z 'rosrun ollamawrapper ollamawrapper.py' Enter
tmux send-keys -t ollama:0.1 C-z 'echo -e "\n\n\nExample CLI service call:\nrosservice call /stt/ollamacall \"input: my name is alice\"\n"' Enter
tmux send-keys -t ollama:0.2 C-z 'rostopic echo /ollama_name' Enter
tmux send-keys -t ollama:0.4 C-z 'rostopic echo /ollama_drink' Enter
tmux send-keys -t ollama:0.3 C-z 'rostopic echo /ollama_confirm' Enter
tmux send-keys -t ollama:0.5 C-z 'rostopic echo /stt/listening' Enter

tmux select-pane -t ollama:0.1