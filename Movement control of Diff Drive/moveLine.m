function [Robot]=moveLine(Robot,vMax,wMax,T,iterTime,wheelsRad,wheelsDist,trace)
    global hvRobot hwRobot hwLeft hwRight world

    Kd = 0.06;
    Kh = 3;

    actualPose=Robot.getPose();
    
    hold on
    subplot(1,2,1)
    title(['Escolha 2 pontos para desenhar a linha'])
    %Get two points to draw the line
    [px,py]=ginput(2);
    title(['Movimento segundo uma linha'])
    hold off
    A = [py(1)-py(2), px(2)-px(1), px(1)*py(2)-px(2)*py(1)]; %Vetor reta (a,b,c) c/ ax+by+c=0
    x = -10000 : 10000;
    m = A(1)/-A(2);
    b = A(3)/-A(2); 
    y = m*x + b;
    hold on;
    plot(x, y)
  
    if(trace)
        hold on
        h=plot(actualPose(1),actualPose(2),'--r');
        uistack(h, 'bottom')
        hold off
    end
    
    Robot=Robot.drawRobot();
    
    finalPose(3) = atan2(-A(1),A(2));
    i = 1;
    while(1)
        % The linear velocity is kept constatn
        vRobot=vMax;

        % Computes the distance from the line
        d = dot(A,[actualPose(1) actualPose(2) 1])/sqrt(A(1)^2 + A(2)^2);
        
        % Gets the angular velocity
        wRobot = (-Kd*d) + Kh*atan2(sin(finalPose(3)-actualPose(3)),cos(finalPose(3)-actualPose(3)));
        if(abs(wRobot)>wMax)
            wRobot=sign(wRobot)*wMax;
        end
        
        %Gets the velocities for the wheels
        w=velRobot2velWheels(vRobot,wRobot,wheelsRad,wheelsDist);
        
        % Gets and updates the robot's next pose
        actualPose=nextPose(T,w,wRobot,actualPose,wheelsRad);
        Robot=Robot.setPose(actualPose);

        if(trace)
            h.XData=[h.XData actualPose(1)];
            h.YData=[h.YData actualPose(2)];
        end
        
        %Checks if robot is out of the map and then updates the map's limit
        if (actualPose(1) < world.XLim(1))
            world.XLim = [world.XLim(1)-100 world.XLim(2)-100];
        end
        
        if (actualPose(2) < world.YLim(1))  
            world.YLim = [world.YLim(1)-100 world.YLim(2)-100];
        end
        
        if (actualPose(1) > world.XLim(2))
            world.XLim = [world.XLim(1)+100 world.XLim(2)+100];
        end
        
        if (actualPose(2) > world.YLim(2))  
            world.YLim = [world.YLim(1)+100 world.YLim(2)+100];
        end
        
        Robot=Robot.drawRobot();
        hvRobot.XData=[hvRobot.XData (i-1)/10];
        hvRobot.YData=[hvRobot.YData vRobot];
        hwRobot.XData=[hwRobot.XData (i-1)/10];
        hwRobot.YData=[hwRobot.YData wRobot];
        hwLeft.XData=[hwLeft.XData (i-1)/10];
        hwLeft.YData=[hwLeft.YData w(1)];
        hwRight.XData=[hwRight.XData (i-1)/10];
        hwRight.YData=[hwRight.YData w(2)];
        
        if(size(Robot.hCoords,1)==0)
            Robot.hCoords=text(actualPose(1)+8,actualPose(2)+8,['d= ',num2str(d)],'Color','b');
        else
            Robot.hCoords.String = ['d= ',num2str(d)];
            Robot.hCoords.Position(1)=actualPose(1)+8;
            Robot.hCoords.Position(2)=actualPose(2)+8;
        end
        
        pause(iterTime);
        i=i+1;
    end
end