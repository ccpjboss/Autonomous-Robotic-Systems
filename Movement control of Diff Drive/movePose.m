function [Robot]=movePose(Robot,vMax,wMax,T,iterTime,wheelsRad,wheelsDist,trace,p)
    global hvRobot hwRobot hwLeft hwRight

    Krho = 0.15;
    Kalpha = 1.5;
    Kbeta = -1;
    
    actualPose=Robot.getPose();
  
    if(trace)
        hold on
        h=plot(actualPose(1),actualPose(2),'--c');
        uistack(h, 'bottom')
        hold off
    end  
    
    Robot=Robot.drawRobot();
    
    hold on
    subplot(1,2,1)
    title(['Escolha a posição e orientação de ' num2str(p) ' poses'])
    %Posição final arbitrária
    [x,y]=ginput(2*p);
    title(['Movimento para uma pose arbitrária'])
    hold off
    d=0;
    for j=1:2:2*p
        d=d+1;
        Theta=atan2(y(j+1)-y(j),x(j+1)-x(j));
        finalPose(:,:,d) = [x(j);y(j);Theta];
        hold on;
        quiver(x(j),y(j),x(j+1)-x(j),y(j+1)-y(j),0);
    end
    
    i=1;
    for j=1:1:d
        %Verificar se precisa de andar em frente ou para trás
        dx = finalPose(1,j)-actualPose(1);
        dy = finalPose(2,j)-actualPose(2);
        alpha = -actualPose(3) + atan2(dy,dx);
        alpha = atan2(sin(alpha),cos(alpha)); %Verifica contradominio
        if (alpha >= -pi/2 && alpha <= pi/2)
            dir = 1; %Flag para saber se anda para a frente(1) ou para trás(-1)
        else
            dir = -1;
        end

        beta = 1;
        rho = 1;
        while (rho > 0.1 || beta > 0.1)
            dx = finalPose(1,j) - actualPose(1);
            dy = finalPose(2,j) - actualPose(2);
            rho = sqrt(dx^2 + dy^2);
            alpha = -actualPose(3) + atan2(dy,dx);
            alpha = atan2(sin(alpha),cos(alpha));

            %Correção do alpha caso ande em marcha atrás
            if (dir == -1) 
                alpha = alpha + pi;
                alpha = atan2(sin(alpha),cos(alpha));
            end

            beta = -actualPose(3) - alpha + finalPose(3,j); %Beta com orientação da pose final
            beta = atan2(sin(beta),cos(beta));

            [rhoNew,alphaNew,betaNew] = updateParameters(rho,alpha,beta,Krho,Kalpha,Kbeta,T);

            vRobot = Krho*rhoNew*dir;
            wRobot = (Kalpha*alphaNew) + (Kbeta*betaNew);

            if(abs(vRobot)>abs(vMax))
                vRobot=vMax*dir;
            end

            if(abs(wRobot)>abs(wMax))
                wRobot=wMax;
            end

            w=velRobot2velWheels(vRobot,wRobot,wheelsRad,wheelsDist);
            actualPose=nextPose(T,w,wRobot,actualPose,wheelsRad);
            Robot=Robot.setPose(actualPose);

            if(trace)
                h.XData=[h.XData actualPose(1)];
                h.YData=[h.YData actualPose(2)];
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
                str={['rho= ',num2str(round(rho,3))],['alpha= ',num2str(round(alpha,3))],['beta= ',num2str(round(beta,3))]};
                Robot.hCoords=text(actualPose(1)+8,actualPose(2)+15,str,'Color','b');
            else
                str={['rho= ',num2str(round(rho,3))],['alpha= ',num2str(round(alpha,3))],['beta= ',num2str(round(beta,3))]};
                Robot.hCoords.String = str;
                Robot.hCoords.Position(1)=actualPose(1)+8;
                Robot.hCoords.Position(2)=actualPose(2)+15;
            end
        
            i=i+1;
            pause(iterTime);
        end
    end
end