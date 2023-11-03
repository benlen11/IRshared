clc;
clf;
clear;

hold on

%general properties
baseTr = eye(4);

%set workspace
axis([-3,3,-3,3,-1,2])

%place background
surf([-3,-3;3,3] ...
    ,[-3,3;-3,3] ...
    ,[-0.01,-0.01;-0.01,-0.01] ...
    ,'CData',imread('concrete.jpg') ...
    ,'FaceColor','texturemap');

%setup environment

%long table, chairs, cooktop, grill
table1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,1.5,0]);
verts = [get(table1,'Vertices'), ones(size(get(table1,'Vertices'),1),1)] * trotz(pi/2);
verts(:,2) = verts(:,2)*2;
set(table1,'Vertices',verts(:,1:3));

table2 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0,0]);
verts = [get(table2,'Vertices'), ones(size(get(table2,'Vertices'),1),1)] * trotz(pi/2);
verts(:,2) = verts(:,2)*2;
set(table2,'Vertices',verts(:,1:3));

%setup dobot magician
r = DobotMagician(true);

% %setup created robot
r2baseTr = baseTr*transl(0.5,0,0.5);
r2 = linearV(r2baseTr);
