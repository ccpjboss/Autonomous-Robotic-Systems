classdef map
    properties
        indexObstacles
        mapSegments
        nObstacles
        hObstacles
    end
    
    methods
        function m=map()
            m.nObstacles=4;
            m.indexObstacles=[1 2 3 4];
            m.mapSegments=[0   0   0   200
                           0   200 200 200
                           200 200 200 0
                           200 0   0   0];
            m.hObstacles=[];
        end
        
        function mReturn=addObstacle(m,points)
            
            [r,c]=size(points);
            
            m.nObstacles=m.nObstacles+1;
            m.indexObstacles=[m.indexObstacles size(m.mapSegments,1)+1];
            
            for i=1:r
                if (i==r)
                    m.mapSegments=[m.mapSegments; points(i,:) points(1,:)];
                else
                    m.mapSegments=[m.mapSegments; points(i,:) points(i+1,:)];
                end
            end
            mReturn=m;            
        end
        
        function error=saveMap(m,filePath)
            if (exist(filePath)~=0)
                error=1;
            else
                error=0;
                fileID = fopen(filePath,'wt');
                
                fprintf(fileID,'%d\n',m.nObstacles);
                
                for i=1:m.nObstacles
                    if(i~=m.nObstacles)
                        fprintf(fileID,'%d ',m.indexObstacles(i));
                    else
                        fprintf(fileID,'%d\n',m.indexObstacles(i));
                    end
                end
                
                for j=1:size(m.mapSegments,1)
                    fprintf(fileID,'%6.2f %6.2f %6.2f %6.2f\n',m.mapSegments(j,:));                
                end
                
                fclose(fileID);
            end
        end
        
        function mReturn=loadMap(m,filePath)
            if (exist(filePath)==0)
                mReturn=0;
            else
                m.nObstacles=0;
                m.indexObstacles=[];
                m.mapSegments=[];
                m.hObstacles=[];

                fileID = fopen(filePath,'rt');
            
                m.nObstacles=fscanf(fileID,'%d',1);
                
                m.indexObstacles=fscanf(fileID,'%d',[1 m.nObstacles]);
                
                i=1;
                while(1)
                    [aux count]=fscanf(fileID,'%f',[1 4]);
                    if(count==0)
                        break
                    end
                    m.mapSegments=[m.mapSegments ; aux];
                    i=i+1;
                end
                
                fclose(fileID);
                mReturn=m;
            end
        end
        
        function mReturn=readMap(m,n)
            xlim([0 200])
            ylim([0 200])
            grid on
            axis square
            if(n>0)
                for i=1:n
                    handler=impoly;
                    points=getPosition(handler);
                    
                    m=m.addObstacle(points);
                    
                    hold on
                    m.hObstacles=fill(points(:,1),points(:,2),'k');
                    xlim([0 200])
                    ylim([0 200])
                    grid on
                    axis square
                    hold off
                end
            end
            mReturn=m;
        end
        
        function mReturn=drawMap(m)
            for i=1:m.nObstacles
                if(i==m.nObstacles)
                    hold on
                    m.hObstacles=[m.hObstacles fill(m.mapSegments(m.indexObstacles(i):end,1),m.mapSegments(m.indexObstacles(i):end,2),'k')];
                    hold off
                else
                    hold on
                    m.hObstacles=[m.hObstacles fill(m.mapSegments(m.indexObstacles(i):m.indexObstacles(i+1)-1,1),m.mapSegments(m.indexObstacles(i):m.indexObstacles(i+1)-1,2),'k')];
                    hold off
                end
            end
            mReturn=m;
        end
        
    end
end
    