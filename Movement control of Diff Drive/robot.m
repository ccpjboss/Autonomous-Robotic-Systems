classdef robot
    properties
        %Robot properties
        worldTrobot 
        %Wheels properties
        nWheels
        robotTwheels
        wheels2Twheels
        wheelsPoints
        hWheels
        %Sonars properties
        nSonars
        robotTsonars
        sonarsPoints
        whiskersPoints
        hWhiskers
        hSonars
        %Platform properties
        platformRadius
        hPlatform
        hCoords
    end
    methods
        function r=robot(rPos,rTheta,nW,wPos,wTheta,wRad,wWid,nS,sPos,sTheta,sLen,sWid,whiskersLen,platRad)
            
            r.worldTrobot=[cos(rTheta) -sin(rTheta) rPos(1)
                           sin(rTheta) cos(rTheta)  rPos(2)
                           0           0            1];
            
            r.nWheels=nW;
            
            i=1;
            r.robotTwheels=[cos(wTheta(i)) -sin(wTheta(i)) wPos(1,i)
                            sin(wTheta(i)) cos(wTheta(i))  wPos(2,i)
                            0              0               1];
            
            r.wheels2Twheels=[1 0 0
                              0 1 0
                              0 0 1];
            
            for i=2:nW
                r.robotTwheels=[r.robotTwheels
                                cos(wTheta(i)) -sin(wTheta(i)) wPos(1,i)
                                sin(wTheta(i)) cos(wTheta(i))  wPos(2,i)
                                0              0               1];
                r.wheels2Twheels=[r.wheels2Twheels
                                  1 0 0
                                  0 1 0
                                  0 0 1];
            end
            
            i=1;
            r.wheelsPoints=[wRad(i)   wRad(i)    -wRad(i)   -wRad(i)
                            wWid(i)/2 -wWid(i)/2 -wWid(i)/2 wWid(i)/2
                            1         1          1          1];
                        
            r.hWheels=[];
                        
            for i=2:nW
                r.wheelsPoints=[r.wheelsPoints
                                wRad(i)   wRad(i)    -wRad(i)   -wRad(i)
                                wWid(i)/2 -wWid(i)/2 -wWid(i)/2 wWid(i)/2
                                1         1          1          1];
            end
            
            r.nSonars=nS;
            
            i=1;
            r.robotTsonars=[cos(sTheta(i)) -sin(sTheta(i)) sPos(1,i)
                            sin(sTheta(i)) cos(sTheta(i))  sPos(2,i)
                            0              0               1];
            for i=2:nS
                r.robotTsonars=[r.robotTsonars
                                cos(sTheta(i)) -sin(sTheta(i)) sPos(1,i)
                                sin(sTheta(i)) cos(sTheta(i))  sPos(2,i)
                                0              0               1];
            end
            
            i=1;
            r.sonarsPoints=[sWid(i)/2 sWid(i)/2  -sWid(i)/2 -sWid(i)/2
                            sLen(i)/2 -sLen(i)/2 -sLen(i)/2 sLen(i)/2
                            1         1          1          1];
            
            r.whiskersPoints=[sWid(i)/2 sWid(i)/2+whiskersLen(i)
                              0         0
                              1         1];
                        
            for i=2:nS
                r.sonarsPoints=[r.sonarsPoints
                                sWid(i)/2 sWid(i)/2  -sWid(i)/2 -sWid(i)/2
                                sLen(i)/2 -sLen(i)/2 -sLen(i)/2 sLen(i)/2
                                1         1          1          1];
                            
                r.whiskersPoints=[r.whiskersPoints
                                  sWid(i)/2 sWid(i)/2+whiskersLen(i)
                                  0         0
                                  1         1];
            end
            
            r.hWhiskers=[];
            r.hSonars=[];
            
            r.platformRadius=platRad;
            
            r.hPlatform=[];
            r.hCoords=[];
        end
        
        function rReturn=drawRobot(r)
            
            hold on
            
            %draw platform
            [x,y]=pol2cart(linspace(0,2*pi,36),r.platformRadius*ones(1,36));
            if(size(r.hPlatform,1)==0)
                r.hPlatform=fill(r.worldTrobot(1,3)+x,r.worldTrobot(2,3)+y,'b');
            else
                r.hPlatform.XData=r.worldTrobot(1,3)+x;
                r.hPlatform.YData=r.worldTrobot(2,3)+y;
            end
            
            for i=1:r.nWheels
                P=r.worldTrobot*r.robotTwheels(((i-1)*3+1):(i*3),:)*r.wheels2Twheels(((i-1)*3+1):(i*3),:)*r.wheelsPoints(((i-1)*3+1):(i*3),:);
                if(size(r.hWheels,1)<i)
                    r.hWheels=[r.hWheels
                               fill(P(1,:),P(2,:),'r')];
                else
                    r.hWheels(i).XData=P(1,:);
                    r.hWheels(i).YData=P(2,:);
                end
            end
            
            %draw sonars
            for i=1:r.nSonars
                P=r.worldTrobot*r.robotTsonars(((i-1)*3+1):(i*3),:)*r.sonarsPoints(((i-1)*3+1):(i*3),:);
                if(size(r.hSonars,1)<i)
                    r.hSonars=[r.hSonars
                               fill(P(1,:),P(2,:),'r')];
                else
                    r.hSonars(i).XData=P(1,:);
                    r.hSonars(i).YData=P(2,:);
                end
                    P=r.worldTrobot*r.robotTsonars(((i-1)*3+1):(i*3),:)*r.whiskersPoints(((i-1)*3+1):(i*3),:);
                    if(size(r.hWhiskers,1)<i)
                        r.hWhiskers=[r.hWhiskers
                                     plot(P(1,:),P(2,:),'-g')];
                    else
                        r.hWhiskers(i).XData=P(1,:);
                        r.hWhiskers(i).YData=P(2,:);
                    end
            end
            
            hold off
            
            rReturn=r;
        end
        
        function pose=getPose(r)
            pose=[r.worldTrobot(1,3)
                  r.worldTrobot(2,3)
                  atan2(r.worldTrobot(2,1),r.worldTrobot(1,1))];
        end
        
        function rReturn=setPose(r,pose)
            r.worldTrobot=[cos(pose(3)) -sin(pose(3)) pose(1)
                           sin(pose(3)) cos(pose(3))  pose(2)
                           0            0             1];
            rReturn=r;
        end
        
        function rReturn=setWheelOri(r,wheelNo,alpha)
            r.wheels2Twheels(((wheelNo-1)*3+1):(wheelNo*3),:)=[cos(alpha) -sin(alpha) 0
                                                               sin(alpha) cos(alpha)  0
                                                               0          0           1];
                                                           
            rReturn=r;
        end
        
        function A=getWhiskersSegments(r)
            A=zeros(r.nSonars,4);
            for i=1:r.nSonars
                P=r.worldTrobot*r.robotTsonars(((i-1)*3+1):(i*3),:)*r.whiskersPoints(((i-1)*3+1):(i*3),:);
                A(i,:)=[P(1,1) P(2,1) P(1,2) P(2,2)];
            end
        end
        
        function result=readSensor(r,m)
             
            out=lineSegmentIntersect(r.getWhiskersSegments(),m.mapSegments);
            
            result=zeros(1,size(out.intNormalizedDistance1To2,1));
            for i=1:size(out.intNormalizedDistance1To2,1)
                for j=1:size(out.intNormalizedDistance1To2,2)
                    if(out.intNormalizedDistance1To2(i,j)>=0 && out.intNormalizedDistance1To2(i,j)<=1 && out.intAdjacencyMatrix(i,j))
                        result(i)=1;
                        break
                    end
                end
            end
        end
        function r=cleanDraw(r)
            
            for i=1:r.nWheels
                r.hWheels=[];
            end
            
            for i=1:r.nSonars
                r.hWhiskers=[];
                r.hSonars=[];
            end
            
            r.hPlatform=[];
            clf
        end
        
    end
end
