function w=velRobot2velWheels(vRobot,wRobot,wheelsRadius,wheelsDistance)
    w(1)=(vRobot-(wheelsDistance/2)*wRobot)/wheelsRadius;
    w(2)=(vRobot+(wheelsDistance/2)*wRobot)/wheelsRadius;
end