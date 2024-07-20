There is two grasping methods:
     1 Grasping the objects using the segmented image box position 
       To simulate that, you need to run the follwing commands in the different command windows
       
       - roslaunch lcastor_grasping picking.launch   # don't run this line when you are working the real robot
       - rosrun lcastor_grasping findObject.py      # you need to install mediapipe to able run it. This part will change with the Niko's perception server's and won't be required the mediapipe
       - rosrun lcastor_grasping getPose.py
       - rosrun lcastor_grasping client.py
       - rosrun lcastor_grasping pickObject.py

    2   In order to run grasping multiple object from specified positions and you can  also use it for all your motions without using /grasp_points publisher

        A publisher should publish 
        '/grasp_points'  in  a PoseArray format 

        To go to objects positions run

        - rosrun lcastor_grasping tiago_moveit_grasp.py  




