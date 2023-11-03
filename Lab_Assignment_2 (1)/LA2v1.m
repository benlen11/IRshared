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
%long table, chairs, etc
table1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0,0]);
verts = [get(table1,'Vertices'), ones(size(get(table1,'Vertices'),1),1)];
verts(:,2) = verts(:,2);
set(table1,'Vertices',verts(:,1:3));

%setup dobot magician
rBaseTr = baseTr*transl(0, 0.25, 0.5)*trotz(pi/2);
rd = DobotMagician(rBaseTr);
qdobot = zeros(1,rd.model.n);
rd.model.plot(qdobot,'nojaxes','noarrow','nowrist','nobase','noshadow','noname','notiles', 'fps', 60,'lightpos', ([0 0 -20]))

%setup items
brick1 = PlaceObject('HalfSizedRedGreenBrick.ply');
vertsBri = get(brick1,'Vertices');
tranVertsBri = [vertsBri, ones(size(vertsBri,1),1)] * transl(0,0,0.5)';
set(brick1,'Vertices',tranVertsBri(:,1:3));

spray1 = PlaceObject('spray.ply');
vertsSpr = get(spray1,'Vertices');
tranVertsSpr = [vertsSpr, ones(size(vertsSpr,1),1)] * transl(0,0.6,0.5)';
set(spray1,'Vertices',tranVertsSpr(:,1:3));

%setup initial guesses
pickSprIG1 = [1.5708    1.3963    0.8290   -0.5236         0];

moveSprIG1 = [2.6180    1.1236    0.8618         0         0];
moveSprIG2 = [3.4034    1.1236    0.8618         0         0];
moveSprIG3 = [3.6652    1.1236    0.8618         0         0];

moveSprIG4 = [-1.2763    0.8236    1.2217         0         0];
moveSprIG5 = [-1.6690    0.8236    1.2217         0         0];
moveSprIG6 = [-2.2580    0.8236    1.2217         0         0];

placeSprIG1 = [1.5708    1.3963    0.8290   -0.5236         0];

%setup translate to pickup spray bottle
transSpr1 = transl(0,0.565,0.6);

%setup translates to move spray bottle
transSpr2{1} = transl(-0.25, 0.40, 0.67);
transSpr2{2} = transl(-0.30, 0.17, 0.67);
transSpr2{3} = transl(-0.25, 0.085, 0.67);
transSpr2{4} = transl(0.30, 0.35, 0.67);
transSpr2{5} = transl(0.30, 0.22, 0.67);
transSpr2{6} = transl(0.23, 0.06, 0.67);

%setup move array for spray bottle
moveSpr{1} = moveSprIG1;
moveSpr{2} = moveSprIG2;
moveSpr{3} = moveSprIG3;
moveSpr{4} = moveSprIG4;
moveSpr{5} = moveSprIG5;
moveSpr{6} = moveSprIG6;

% teach mode
% rd.model.teach;

%setup parameters
steps = 100;

%solve IK to pickup spray bottle
qdPi1 = rd.model.getpos;
qdPi2 = rd.model.ikcon(transSpr1, pickSprIG1);
qdPi2(4) = deg2rad(30);
qMatrix1 = jtraj(qdPi1,qdPi2,steps);

%animate pickup spray bottle
for i = 1:1:steps
    rd.model.animate(qMatrix1(i,:));
end

%animate dobot & spray bottle moving to spray points 1-3
for j = 1:1:6
    qdM1 = rd.model.getpos;
    qdM2 = rd.model.ikcon(transSpr2{j}, moveSpr{j});
    qdM2(4) = deg2rad(30);
    qMatrix2 = jtraj(qdM1,qdM2,steps);

    for i = 1:1:steps
        rd.model.animate(qMatrix2(i,:));

        % Animate spray bottle moving with arm
        trSpr = rd.model.fkine(rd.model.getpos);

        rotVertsSpr = (trSpr.R() * vertsSpr')';
        tranVertsSpr = get(spray1,'Vertices');
        tranVertsSpr = [rotVertsSpr, ones(size(vertsSpr,1),1)] * trSpr.T';
        % tranVertsSpr = [vertsSpr, ones(size(vertsSpr,1),1)] * trSpr.T';
        set(spray1,'Vertices',tranVertsSpr(:,1:3));

        drawnow();
    end

    %animate return to start
    % for i = 1:1:steps
    %     rd.model.animate(qMatrix1(i,:));
    % end
end