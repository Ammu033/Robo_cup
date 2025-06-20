# Run as `source connect_tiago.sh TIAGO_NUM ROBOT_IFACE`
# where:
#   - TIAGO_NUM: is the ID number of the robot you are connected
#   - ROBOT_IFACE: is the network interface connected to the robot (e.g. wlan0, eth0...)


TIAGO_NUM=$1
#echo $TIAGO_NUM
#echo $ROBOT_IFACE

if [ ${TIAGO_NUM} -eq 29 ]
then
  TIAGO_NUM=102
elif [ ${TIAGO_NUM} -eq 89 ]
then
  TIAGO_NUM=106
elif [ ${TIAGO_NUM} -eq 125 ]
then
  TIAGO_NUM=105
else
  echo "You entered a wrong TIAGo ID (accepted value: 29,89,125)."
fi

# Change ROS Master
export ROS_MASTER=10.8.0.${TIAGO_NUM}
export ROS_MASTER_URI=http://${ROS_MASTER}:11311
rostopic list &>/dev/null
RETVAL=$?
if [ $RETVAL -ne 0 ]; then
    echo "[ERROR] connection with ROS MASTER not enstablished"
else
    echo "[OK] Connected to ROS MASTER"
fi

# Add forwarding address to DNS
#echo $ROBOT_IFACE
#THIS_IP=`ifconfig ${ROBOT_IFACE} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
THIS_IP=`ip route get ${ROS_MASTER} | grep "src" | sed 's/.*src \([0-9\.]*\).*/\1/'`
export ROS_HOSTNAME=${THIS_IP}
export ROS_IP=${THIS_IP}

sshpass -p "palroot" ssh root@${ROS_MASTER} "addLocalDns -u \"${HOSTNAME}\" -i \"${THIS_IP}\""

