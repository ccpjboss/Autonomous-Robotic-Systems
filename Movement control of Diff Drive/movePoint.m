function [Robot]=movePoint(Robot,vMax,wMax,T,errorMax,iterTime,wheelsRad,wheelsDist,trace,p)
    global hvRobot hwRobot hwLeft hwRight

    Kv = 0.4;
    Ks = 0.75;
    
    actualPose=Robot.getPose();
    
%     hold on
    subplot(1,2,1)
    title(['Escolha ' num2str(p) ' pontos no mapa'])
    %Escolher o Ponto
    [x,y]=ginput(p);
    title(['Movimento para um ponto'])
    hold off
    for j=1:1:p
        finalPose(:,:,j)= [x(j);y(j);0];
        hold on;
        plot(x(j), y(j), 'ro', 'MarkerSize', 30);
        plot(x(j), y(j), '+', 'MarkerSize', 5);
        text(x(j)-30,y(j)-10,['(',num2str(round(x(j),3)),' , ',num2str(round(y(j),3)),')'],'Color','r')
    end
    
    if(trace)
        hold on
        h=plot(actualPose(1),actualPose(2),'--c');
        uistack(h, 'bottom')
        hold off
    end
    
    i=1;
    j=1;
    error=1;    
    Robot=Robot.drawRobot();
    for j=1:1:p
        error=1;
        while(abs(error)>errorMax)
            error=norm(finalPose(1:2,j)-actualPose(1:2)); %Gets the error

            % Computes the linear velocity
            vRobot = Kv*sqrt((finalPose(1,j)-actualPose(1))^2 + (finalPose(2,j)-actualPose(2))^2);
            if(abs(vRobot)>abs(vMax))
                vRobot=vMax;
            end

            % Computes angular velocity
            finalPose(3,j) = atan2(finalPose(2,j)-actualPose(2),finalPose(1,j)-actualPose(1));
            wRobot = Ks*atan2(sin(finalPose(3,j)-actualPose(3)),cos(finalPose(3,j)-actualPose(3)));
            if(abs(wRobot)>wMax)
                wRobot=sign(wRobot)*wMax;
            end

            % Gets the velocities for the wheels
            w=velRobot2velWheels(vRobot,wRobot,wheelsRad,wheelsDist);

            % Gets the next pose
            actualPose=nextPose(T,w,wRobot,actualPose,wheelsRad);
            Robot=Robot.setPose(actualPose);

            if(trace)
                h.XData=[h.XData actualPose(1)];
                h.YData=[h.YData actualPose(2)];
            end

            Robot=Robot.drawRobot();

            % Plots the velocities of the robot
            hvRobot.XData=[hvRobot.XData (i-1)/10];
            hvRobot.YData=[hvRobot.YData vRobot];
            hwRobot.XData=[hwRobot.XData (i-1)/10];
            hwRobot.YData=[hwRobot.YData wRobot];
            hwLeft.XData=[hwLeft.XData (i-1)/10];
            hwLeft.YData=[hwLeft.YData w(1)];
            hwRight.XData=[hwRight.XData (i-1)/10];
            hwRight.YData=[hwRight.YData w(2)];
            
            % Plots the error of the robot in real time
            if(size(Robot.hCoords,1)==0)
                Robot.hCoords=text(actualPose(1)+8,actualPose(2)+8,['erro= ',num2str(error)],'Color','b');
            else
                Robot.hCoords.String = ['erro= ',num2str(error)];
                Robot.hCoords.Position(1)=actualPose(1)+8;
                Robot.hCoords.Position(2)=actualPose(2)+8;
            end

            pause(iterTime);
            i=i+1;
        end
    end
end