#!/usr/bin/env python
import rospy, os, re, subprocess
from std_msgs.msg import Int8


print(rospy.get_param('/volume/control', 'Master'))

rospy.init_node("volume")

# Subtract or add to the volume percentage by the value published on '/volume/percentChange'
def vcb(data):
    control = rospy.get_param('/volume/control', 'Master')
    print(abs(int(data.data)))
    if data.data >= 0:
        os.system("amixer -D pulse sset " + control  + " " + str(data.data) + "%+")
    else:
        os.system("amixer -D pulse sset " + control + " " + str(abs(int(data.data))) + "%-")


vSub = rospy.Subscriber("/volume/percentChange", Int8, callback=vcb)

# Every .25 seconds, search for the volume percentage in the string 
# output of 'amixer get PCM', convert to Int8 and publish to
# '/volume/percent' if value changes
volPercentPub = rospy.Publisher('volume/percent', Int8, queue_size=1, latch=True)
volPercentRE = re.compile("(?<=\[)[0-9]+(?=%\])")

prevVolPercent = None
while not rospy.is_shutdown():
    control = rospy.get_param('/volume/control', 'Master')
    amixerOutput = subprocess.check_output(["amixer", "get", control])
    volumePercent = Int8(int(volPercentRE.search(amixerOutput).group(0)))
    if volumePercent != prevVolPercent:
        volPercentPub.publish(volumePercent)
    prevVolPercent = volumePercent
    rospy.sleep(0.25)
