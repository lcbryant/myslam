mv /home/lcbryant/catkin_ws/src/MYNT-EYE-ORB-SLAM2-Sample/keyframeId.txt /home/lcbryant/桌面/myslam/image
mv /home/lcbryant/catkin_ws/src/MYNT-EYE-ORB-SLAM2-Sample/CameraTrajectory.txt t /home/lcbryant/桌面/myslam/result
cd /home/lcbryant/桌面/myslam/result
mv CameraTrajectory.txt pos_0.txt
cd build
./main
