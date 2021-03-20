clear
close all
clc

global hvRobot hwRobot hwLeft hwRight world
fprintf('1-> Point Move\n');
fprintf('2-> Line Move\n');
fprintf('3-> Pose Move\n');
c = input('Your option: ');

if c == 1
    prompt = "Quantos pontos quer?";
    p = input(prompt);
elseif c == 3
    prompt = "Quantas poses quer?";
    p = input(prompt);
end
  
    

m=map();
filename=input('Insert the filename:','s');
m=m.loadMap(filename);
figure('units','normalized','outerposition',[0 0 1 1])
m=m.drawMap();

world=subplot(1,2,1);
xlim auto
ylim auto
xlim([0 200]);
ylim([0 200]);
axis square
grid on

subplot(4,2,2)
hvRobot=plot(0,0);
title('Linear Velocity of Robot')
grid on
subplot(4,2,4)
hwRobot=plot(0,0);
grid on
title('Angular Velocity of Robot')
subplot(4,2,6)
hwLeft=plot(0,0);
grid on
title('Angular Velocity of Left Wheel')
subplot(4,2,8)
hwRight=plot(0,0);
grid on
title('Angular Velocity of Right Wheel')

hold on
subplot(1,2,1)
title(['Escolha a posição e orientação do robô'])
[x,y]=ginput(2);
rPos=[x(1) y(1)]';
rTheta=atan2(y(2)-y(1),x(2)-x(1));
hold off

%Wheels Properties
nW=2;
wPos=[0   0
      4.5 -4.5];
wTheta=[0 0];

wRad=[2 2];
wWid=[1.5 1.5];

%Sensors Properties
nS=3;
sPos=[5.2 6 5.2
      3   0 -3];
sTheta=[pi/6 0 -pi/6];
sLen=[2 2 2];
sWid=[1 1 1];
whiskersLen=[5 5 5];

%Platform Properties
platRad=6;

Robot=robot(rPos,rTheta,nW,wPos,wTheta,wRad,wWid,nS,sPos,sTheta,sLen,sWid,whiskersLen,platRad);
pose=Robot.getPose();

Robot=Robot.drawRobot();

T=0.1;
iterTime=0.01;
errorMaxPos=1e-1;
errorMaxAngle=1e-1;
vMax=5;
dist=50;
wMax=pi/10;
wheelsRad=2;
wheelsDist=9;

switch c
    case 1
        Robot=movePoint(Robot,vMax,wMax,T,errorMaxPos,iterTime,wheelsRad,wheelsDist,1,p);
    case 2
        Robot=moveLine(Robot,vMax,wMax,T,iterTime,wheelsRad,wheelsDist,1);
    case 3
        Robot=movePose(Robot,vMax,wMax,T,iterTime,wheelsRad,wheelsDist,1,p);
end