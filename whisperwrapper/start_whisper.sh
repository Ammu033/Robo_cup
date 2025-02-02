# to be run on the *HOST* (the dell laptop)

echo 'robocup' | sudo -S docker compose -f ~/noetic-llama/docker-compose.yaml up -d
curl http://localhost:11434/api/pull -d '{"name": "nexusraven:13b-v2-q2_K"}'
curl http://localhost:11434/api/pull -d '{"name": "llama3.1"}'
curl http://localhost:11434/api/pull -d '{"name": "llava:7b"}'
curl http://localhost:11434/api/pull -d '{"name": "deepseek-r1:8b"}'
curl http://localhost:11434/api/pull -d '{"name": "deepseek-r1:14b"}'

tmux new-session -s whisper -d  "cd ~/noetic-llama/whisper_ws && catkin_make && source devel/setup.bash && bash"

tmux split-window -v -t whisper:0
tmux split-window -h -t whisper:0.1

sleep 3
tmux send-keys -t whisper:0.2 C-z 'rostopic echo /stt/transcription' Enter
tmux send-keys -t whisper:0.1 C-z 'echo -e "\n\nTo manually set whisper to listen:\nrostopic pub /stt/listening ollamamessages/WhisperListening \"listening: True\" -1"\n\n' Enter
sleep 1
tmux send-keys -t whisper:0.1 C-z 'rostopic pub /stt/listening ollamamessages/WhisperListening "listening: True" -1'
sleep 10
tmux send-keys -t whisper:0.0 C-z 'source ~/noetic-llama/whisper_ws/devel/setup.bash' Enter
tmux send-keys -t whisper:0.0 C-z 'rosrun whisperwrapper whisperwrapper.py' Enter
