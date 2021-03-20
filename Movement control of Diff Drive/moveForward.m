function [Robot ,travDist]=moveForward(Robot,m,dist,vMax,T,errorMax,iterTime,wheelsRad,wheelsDist,trace)
    travDist=0;
    Kp=vMax/(0.5*dist);
    
    actualPose=Robot.getPose();
    initialPose=actualPose;
    finalPose=actualPose+[sign(vMax)*dist*cos(actualPose(3)) sign(vMax)*dist*sin(actualPose(3)) 0]';
    
    if(trace)
        hold on
        h=plot(actualPose(1),actualPose(2),'--c');
        uistack(h, 'bottom')
        hold off
    end
    error=1;    
    Robot=Robot.drawRobot();
    while(abs(error)>errorMax)
        error=norm(finalPose(1:2)-actualPose(1:2));
        
        vRobot=Kp*error;
        if(abs(vRobot)>abs(vMax))
            vRobot=vMax;
        end
        
        w=velRobot2velWheels(vRobot,0,wheelsRad,wheelsDist);
        lastPose=actualPose;
        actualPose=nextPose(T,w,0,actualPose,wheelsRad,wheelsDist);
        
        %=========Colision Verification===================================%
        Robot=Robot.setPose(actualPose);
        sensors=Robot.readSensor(m);
        for i=1:size(sensors,2)
            if(sensors(i))
                Robot=Robot.setPose(lastPose);
                travDist=norm(lastPose(1:2)-initialPose(1:2));
                return
            end
        end
        %=================================================================%
        if(trace)
            h.XData=[h.XData actualPose(1)];
            h.YData=[h.YData actualPose(2)];
        end
        
        Robot=Robot.drawRobot();
        pause(iterTime);
    end

end