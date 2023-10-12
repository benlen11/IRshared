clc;
clf;
clear;

hold on

%general properties
baseTr = eye(4);

%set workspace
axis([-2,2,-2,2,-1,2])

%place background
surf([-3,-3;3,3] ...
    ,[-2,2;-2,2] ...
    ,[-0.01,-0.01;-0.01,-0.01] ...
    ,'CData',imread('concrete.jpg') ...
    ,'FaceColor','texturemap');

%setup environment

%long table, chairs, cooktop, grill
table = PlaceObject('barrier1.5x0.2x1m.ply',[0,1,0]);
verts = [get(fence1,'Vertices'), ones(size(get(fence1,'Vertices'),1),1)]* trotz(pi/2);
verts(:,2) = verts(:,2)*2;
set(fence1,'Vertices',verts(:,1:3));

%setup dobot magician
r = DobotMagician(true);

% %setup created robot
r2baseTr = baseTr*transl(0.5,0,0.2);
r2 = DVince(r2baseTr);