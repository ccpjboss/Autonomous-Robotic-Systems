function pose=nextPose(T,w,wRobot,poseRobot,wheelsRadius)
%NEXTPOSE Calculates the next pose of the robot using cinematic equations
if wRobot == 0
    pose =[poseRobot(1)+T*wheelsRadius*((w(2)+w(1))/2)*cos(poseRobot(3))
           poseRobot(2)+T*wheelsRadius*((w(2)+w(1))/2)*sin(poseRobot(3))
           poseRobot(3)];
else
    pose =[poseRobot(1)+T*wheelsRadius*((w(2)+w(1))/2)*sin(wRobot*T/2)/(wRobot*T/2)*cos(poseRobot(3)+wRobot*T/2)
           poseRobot(2)+T*wheelsRadius*((w(2)+w(1))/2)*sin(wRobot*T/2)/(wRobot*T/2)*sin(poseRobot(3)+wRobot*T/2)
           poseRobot(3)+wRobot*T];
end
end