clc;
clf;
clear;
% close all;

hold on

%% setup environment

% setup workspace
axis([-3,3,-3,3,-0.1,3]);
set(gca, 'Projection', 'orthographic');
axis equal;

% Texture images
textures = {'carpet.jpg', 'cityskyline.jpg', 'woodslats.jpg' 'water.jpg'};

% Rotate the cityskyline image by 90 degrees counterclockwise
cityskyline_upright = imrotate(imread(textures{2}), 270);
slats_upright = imrotate(imread(textures{3}), 270);

% floor
surface([-3,3; -3,3], [-3,-3; 3,3], [-0.01,-0.01; -0.01,-0.01], 'CData', imread(textures{1}), 'FaceColor', 'texturemap');

% left wall (city skyline)
surface([-3,-3; 3,3], [3,3; 3,3], [-0.01,3; -0.01,3], 'CData', cityskyline_upright, 'FaceColor', 'texturemap');

% front wall
surface([3,3; 3,3], [-3,-3; 3,3], [-0.01,3; -0.01,3], 'CData', slats_upright, 'FaceColor', 'texturemap');

% water
surface([-2.3,-1.7; -2.3,-1.7], [1.1,1.1; 0.7,0.7], [0.51,0.51; 0.51,0.51], 'CData', imread(textures{4}), 'FaceColor', 'texturemap');

%% setup environmental items
table1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[-0.25,0.6,0]);
vertstab = [get(table1,'Vertices'), ones(size(get(table1,'Vertices'),1),1)];
vertstab(:,1) = vertstab(:,1)*1.25;
vertstab(:,2) = vertstab(:,2)*1.25;
set(table1,'Vertices',vertstab(:,1:3));

table2 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[-4,0.75,0]);
vertstab2 = [get(table2,'Vertices'), ones(size(get(table2,'Vertices'),1),1)];
vertstab2(:,1) = vertstab2(:,1)*0.5;
set(table2,'Vertices',vertstab2(:,1:3));

basin = PlaceObject('Sink.ply',[-2,0.9,0.45]);
vertsbas = [get(basin,'Vertices'), ones(size(get(basin,'Vertices'),1),1)];
set(basin,'Vertices',vertsbas(:,1:3));

% fence = PlaceObject('fenceFinal.ply');
% vertfen = get(fence,'Vertices');
% vertfen(:,1) = vertfen(:,1)*3;
% vertfen(:,2) = vertfen(:,2)*6;
% tranVertfen = [vertfen, ones(size(vertfen,1),1)] * transl(1,-0.5,0)' * trotz(pi/2);
% set(fence,'Vertices',tranVertfen(:,1:3));

fire = PlaceObject('fireExtinguisher.ply',[1, -1.25, 0]);
vertsfire = [get(fire,'Vertices'), ones(size(get(fire,'Vertices'),1),1)];
set(fire,'Vertices',vertsfire(:,1:3));

emer = PlaceObject('emergencyStopWallMounted.ply',[1.5, -1, 0.5]);
vertsemer = [get(emer,'Vertices'), ones(size(get(emer,'Vertices'),1),1)];
set(emer,'Vertices',vertsemer(:,1:3));

emer2 = PlaceObject('emergencyStopWallMounted.ply');
vertemer = get(emer2,'Vertices');
vertemer(:,1) = vertemer(:,1);
vertemer(:,2) = vertemer(:,2);
tranVertemer = [vertemer, ones(size(vertemer,1),1)] * transl(-1.63, 1.2, 0.5)' * trotz(pi);
set(emer2,'Vertices',tranVertemer(:,1:3));

pers = PlaceObject('personMaleOld.ply');
vertpers = get(pers,'Vertices');
tranVertpers = [vertpers, ones(size(vertpers,1),1)] * transl(-2.5,2,0)' * trotz(pi);  % outside light curtain
% tranVertpers = [vertpers, ones(size(vertpers,1),1)] * transl(-2.5,1.3,0)' * trotz(pi); % in light curtain
set(pers,'Vertices',tranVertpers(:,1:3));
% set(pers,'Vertices',tranVertpers(:,1:3), 'Facecolor', 'yellow');

% Person collision detection
xperC = tranVertpers(:, 1);
yperC = tranVertpers(:, 2);
zperC = tranVertpers(:, 3);

heightperC = max(zperC) - min(zperC);
centerZperC = (max(zperC) + min(zperC)) / 2;

% radiiperC = sqrt(xperC.^2 + yperC.^2);
% radiusperC = max(radiiperC);

persCol = collisionCylinder(0.5, heightperC);

poseperC = eye(4);
poseperC(1:3, 4) = [mean(xperC); mean(yperC); centerZperC];
persCol.Pose = poseperC;
% End of person collision detection

% box1 = collisionBox(0.5, 2, 1);
% poseBox1 = transl(0.5,0.75,0.55);
% box1.Pose = poseBox1;
% show(box1);

lightC1 = collisionCylinder(0.05, 4);
poseLC1 = transl(2,-1,0);
lightC1.Pose = poseLC1;
show(lightC1);

lightC2 = collisionCylinder(0.05, 4);
poseLC2 = transl(2.3,-1,0);
lightC2.Pose = poseLC2;
show(lightC2);

lightC3 = collisionCylinder(0.05, 4);
poseLC3 = transl(2.6,-1,0);
lightC3.Pose = poseLC3;
show(lightC3);

lightC4 = collisionCylinder(0.05, 4);
poseLC4 = transl(2.9,-1,0);
lightC4.Pose = poseLC4;
show(lightC4);

floorCol = collisionBox(6,4,0.1);
poseFloor = transl(0,1,0);
floorCol.Pose = poseFloor;
% show(floorCol);

% centerpnt = [1,0.7,0.55];
% side = 1.5;
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% basin2 = PlaceObject('Sink.ply',[-1.3,-1.15,0.55]);
% vertsbas = [get(basin2,'Vertices'), ones(size(get(basin2,'Vertices'),1),1)]*trotz(pi/2);
% set(basin2,'Vertices',vertsbas(:,1:3));

%% setup movement items
sponge = PlaceObject('spongeb.ply');
vertspo = get(sponge,'Vertices');
tranVertspo = [vertspo, ones(size(vertspo,1),1)] * transl(-2,0.8,0.5)';
set(sponge,'Vertices',tranVertspo(:,1:3), 'Facecolor', 'yellow');

%% setup robots

baseTr = eye(4);

% setup dobot magician
rDBaseTr = baseTr*transl(-2, 0.5, 0.5);
rd = DobotMagician(rDBaseTr);
qdobot = zeros(1,rd.model.n);
rd.model.plot(qdobot,'nojaxes','noarrow','nowrist','nobase','noshadow','noname','notiles', 'fps', 60 ,'lightpos', ([0 0 -20]))

%setup TM12
rlBaseTr = baseTr*transl(-0.5, 0.75, 0.5);
rl = UR10(rlBaseTr);
rl.model.tool = troty(pi);
qlinear = zeros(1,rl.model.n);
qlinear(2) = -pi/2;
qlinear(4) = -pi/2;
qlinear(5) = -pi/2;
rl.model.plot(qlinear,'nojaxes','noarrow','nowrist','nobase','noshadow','noname','notiles', 'fps', 60 ,'lightpos', ([0 0 -20]))

% Add Ellipsoid to TM12
capsuleLink4 = collisionSphere(1);
capsuleLink5 = collisionCapsule(0.1,0.12);
capsuleLink6 = collisionCapsule(0.1,0.05);

% T1 = getLinkTransform(rl, qlinear, linkNumber1);
% T2 = getLinkTransform(rl, qlinear, linkNumber2);
% T3 = getLinkTransform(rl, qlinear, linkNumber3);


%% setup pick and place arrays for Dobot

% setup initial guesses for Dobot
pickSpoDIG1 = [1.5708    1.2254    0.6327    0.7854         0];

moveSpoDIG1 = [1.5708    1.2600    0.6327    0.7854         0];
moveSpoDIG2 = [1.5708    1.2254    0.6327    0.7854         0];
moveSpoDIG3 = [1.5708    1.2600    0.6327    0.7854         0];
moveSpoDIG4 = [1.5708    1.2254    0.6327    0.7854         0];

placeSpoDIG1 = [0    1.2599    0.6327    1.1781         0];

homeDIG1 = [0 0 0 0 0];

% initial guess array for Dobot
moveD{1} = moveSpoDIG1;
moveD{2} = moveSpoDIG2;
moveD{3} = moveSpoDIG3;
moveD{4} = moveSpoDIG4;

% setup translate to pickup sponge by Dobot
transSpoD1 = transl(-2,0.845,0.565);

% setup translate to move sponge by Dobot
transSpoD2{1} = transl(-2,0.845,0.535);
transSpoD2{2} = transl(-2,0.845,0.565);
transSpoD2{3} = transl(-2,0.845,0.535);
transSpoD2{4} = transl(-2,0.845,0.635);

% setup translate to place sponge by Dobot
transSpoD3 = transl(-1.669,0.5,0.55);

% setup translate to home Dobot
transHoD1 = transl(-2.06, 0.5, 0.971);

% setup translate to home TM12
transHoU1 = transl(-0.5904, 0.5861, 1.928);

% setup initial guess for TM12
pickSpoUIG1 = [-0.0982   -0.4909    0.8836   -1.9635   -1.2763         0];

placeSpoUIG1 = [0.5890   -0.5890    0.7854   -1.5708   -1.5708         0];

homeUIG1 = [0 -90 0 -90 -90 0];

% setup translate to pickup sponge by TM12
transSpoU1 = transl(-1.6952, 0.49, 0.55);

% setup translate to move sponge by TM12
transSpoU2 = transl(-1.5, 0, 0.55);

transSpoU4 = transl(-1.5, 1.5, 0.55);

% setup initial guess for TM12 pass 1
moveSpoUIG{1} = [0.5890   -0.5890    0.7854   -1.5708   -1.5708         0];
moveSpoUIG{2} = [2.4166   -0.3955    0.5712   -1.5708   -1.5708         0];
moveSpoUIG{3} = [2.4166   -0.3955    0.5712   -1.5708   -1.5708         0];
moveSpoUIG{4} = [0.5890   -0.5890    0.7854   -1.5708   -1.5708         0];

% setup initial guess for TM12 pass 2
moveSpoUIG{5} = [-0.6981   -0.6981    1.0472   -1.5708   -1.5708         0];
moveSpoUIG{6} = [-2.8424   -0.6981    1.0472   -1.5708   -1.5708         0];
moveSpoUIG{7} = [-2.8424   -0.6981    1.0472   -1.5708   -1.5708         0];
moveSpoUIG{8} = [-0.6981   -0.6981    1.0472   -1.5708   -1.5708         0];

% setup RMRC translate pass 1
transSpoU3{1} = transl(-1.5, 0, 0.55);
transSpoU3{2} = transl(0.5, -0.2, 0.55);
transSpoU3{3} = transl(0.5, -0.1, 0.55);
transSpoU3{4} = transl(-1.8, 0, 0.55);

% setup RMRC translate pass 2
transSpoU3{5} = transl(-1.5, 1.7, 0.55);
transSpoU3{6} = transl(0.5, 1.8, 0.55);
transSpoU3{7} = transl(0.5, 1.7, 0.55);
transSpoU3{8} = transl(-1.5, 1.8, 0.55);

%% person collision
isCollidingPersFloor = checkCollision(persCol, floorCol);
isCollidingPersLC1 = checkCollision(persCol, lightC1);
isCollidingPersLC2 = checkCollision(persCol, lightC2);
isCollidingPersLC3 = checkCollision(persCol, lightC3);
isCollidingPersLC4 = checkCollision(persCol, lightC4);
% show(persCol);
% show(floorCol);

if isCollidingPersFloor || isCollidingPersLC1 || isCollidingPersLC2 || isCollidingPersLC3 || isCollidingPersLC4
            error('Person in operating area');
        else
            disp('All clear');
        end 

%% animate dobot
% setup parameters
steps = 3;
stepslow = 35;

% solve IK to pickup sponge by Dobot
qPiD1 = rd.model.getpos;
qPiD2 = rd.model.ikcon(transSpoD1, pickSpoDIG1);
qMatrixD1 = jtraj(qPiD1,qPiD2,stepslow);

% animate Dobot pickup sponge
for i = 1:1:stepslow
    rd.model.animate(qMatrixD1(i,:));
end

% animate Dobot & sponge moving
for j = 1:1:4
    qMoD1 = rd.model.getpos;
    qMoD2 = rd.model.ikcon(transSpoD2{j}, moveD{j});
    qMatrixD2 = jtraj(qMoD1,qMoD2,stepslow);

    for i = 1:1:stepslow
        rd.model.animate(qMatrixD2(i,:));

        % Animate sponge moving with arm
        trD = rd.model.fkine(rd.model.getpos);

        offset = [0, 0, -0.06];

        tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trD.T' * transl(offset)';
        set(sponge,'Vertices',tranVertspo(:,1:3));
    end
end

% solve IK to place sponge by Dobot
qPlD1 = rd.model.getpos;
qPlD2 = rd.model.ikcon(transSpoD3*rpy2tr(0,0,-pi/2), placeSpoDIG1);
qMatrixD3 = jtraj(qPlD1,qPlD2,stepslow);

% animate sponge placement by Dobot
for i = 1:1:stepslow
    rd.model.animate(qMatrixD3(i,:));

    % Animate sponge moving with arm
    trD = rd.model.fkine(rd.model.getpos);

    offset = [0, 0, -0.06];

    tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trD.T' * transl(offset)';
    set(sponge,'Vertices',tranVertspo(:,1:3));
end

% solve IK to home Dobot
qHoD1 = rd.model.getpos;
qHoD2 = rd.model.ikcon(transHoD1, homeDIG1);
qMatrixD3 = jtraj(qHoD1,qHoD2,stepslow);

% animate Dobot home
for i = 1:1:stepslow
    rd.model.animate(qMatrixD3(i,:));
end

%% animate TM12
% solve IK to pickup sponge by TM12
qPiU1 = rl.model.getpos;
qPiU2 = rl.model.ikcon(transSpoU1, pickSpoUIG1);
qMatrixU1 = jtraj(qPiU1,qPiU2,steps);

% animate TM12 pickup sponge
for i = 1:1:steps
    rl.model.animate(qMatrixU1(i,:));
end

% solve IK to place sponge by TM12
qPlU1 = rl.model.getpos;
qPlU2 = rl.model.ikcon(transSpoU2*rpy2tr(0,0,-pi/2), placeSpoUIG1);
qMatrixU2 = jtraj(qPlU1,qPlU2,steps);

% animate sponge placement by TM12
for i = 1:1:steps
    rl.model.animate(qMatrixU2(i,:));

    % Animate sponge moving with arm
    trU = rl.model.fkine(rl.model.getpos);

    % offset = [0, 0, -0.06];

    tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trU.T' * transl(offset)';
    set(sponge,'Vertices',tranVertspo(:,1:3));
end

% RMRC animate
stepR = 20;
deltaT = 0.05;

qU1 = [qMatrixU2(3,:)];

for j = 1:1:4-1
    tranSpoUplus = transSpoU3{(j+1)};
    moveSpoUIGplus = moveSpoUIG{(j+1)};
    qU2 = rl.model.ikcon(tranSpoUplus, moveSpoUIGplus);
    % qU2 = rl.model.ikcon(transSpoU3{2}, moveSpoUIG{2});
    qMatrixU3 = jtraj(qU1, qU2, stepR);

    x1 = transSpoU3{j}(1:3, 4);
    x2 = tranSpoUplus(1:3, 4);
    x = zeros(3, stepR);
    s = lspb(0, 1, stepR);
    for i = 1:stepR
        x(:, i) = x1*(1-s(i)) + s(i)*x2;
    end

    qMatrixU3 = nan(stepR, 6);
    qMatrixU3(1, :) = qU1;

    for i = 1:1:stepR-1
        xdot_linear = (x(:, i+1) - x(:, i)) / deltaT;
        xdot_angular = [0; 0; 0];
        xdot = [xdot_linear; xdot_angular];

        J = rl.model.jacob0(qMatrixU3(i, :));
        qdot = pinv(J)*xdot;
        qMatrixU3(i+1, :) = qMatrixU3(i, :) + deltaT*qdot';

        
    end

    for i = 1:1:stepR
        rl.model.animate(qMatrixU3(i,:));

        % Collision detection section
        allTransforms = cell(1, rl.model.n);  % Pre-allocate a cell array for the transformations
        currentT = eye(4);  % Start with the identity matrix

        qT = qMatrixU3(i,:);

        % for k = 1:rl.model.n
        %     % Compute the transformation for the current link
        %     ai = rl.model.A(k, qT);
        % 
        %     % Extract the matrix representation from the SE3 object
        %     amatrix = ai.double;
        % 
        %     % Update the cumulative transformation
        %     currentT = currentT * amatrix;
        % 
        %     % Store the transformation in the cell array
        %     allTransforms{k} = currentT;
        % end

        linkT6 = rl.model.fkine(qT);

        % linkT6 = allTransforms{6};
        % linkT5 = allTransforms{5};
        % linkT4 = allTransforms{4};

        capsuleLink6.Pose = linkT6.T;
        % capsuleLink5.Pose = linkT5;
        % capsuleLink4.Pose = linkT4;

        % show(capsuleLink4);
        % show(capsuleLink5);
        % show(capsuleLink6);

        % Animate sponge moving with arm
        trU = rl.model.fkine(rl.model.getpos);

        offset = [0, 0, -0.06];

        tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trU.T' * transl(offset)';
        set(sponge,'Vertices',tranVertspo(:,1:3));
        % end of sponge animation

        % isColliding6 = checkCollision(capsuleLink6, box1);
        % % isColliding5 = checkCollision(capsuleLink5, box1);
        % % isColliding4 = checkCollision(capsuleLink4, box1);

        % if isColliding6 || isColliding5 || isColliding4
        % if isColliding6
        %     error('Collision detected');
        % else
        %     disp('No collisions detected');
        % end
    end
    qU1 = qMatrixU3(end, :);
end

% solve IK to home TM12
qHoU1 = rl.model.getpos;
qHoU2 = homeUIG1;
qMatrixU4 = jtraj(qHoU1,qHoU2,steps);

% animate TM12 home
for i = 1:1:steps
    rl.model.animate(qMatrixU4(i,:));
end

%% animate Dobot again
% solve IK to pickup sponge by Dobot
qPiD3 = rd.model.getpos;
qPiD4 = rd.model.ikcon(transSpoD3, placeSpoDIG1);
qMatrixD4 = jtraj(qPiD3,qPiD4,stepslow);

% animate Dobot pickup sponge
for i = 1:1:stepslow
    rd.model.animate(qMatrixD4(i,:));
end

% solve IK to place sponge by Dobot
qPlD3 = rd.model.getpos;
qPlD4 = rd.model.ikcon(transSpoD1*rpy2tr(0,0,-pi/2), pickSpoDIG1);
qMatrixD5 = jtraj(qPlD3,qPlD4,stepslow);

% animate sponge placement by Dobot
for i = 1:1:stepslow
    rd.model.animate(qMatrixD5(i,:));

    % Animate sponge moving with arm
    trD = rd.model.fkine(rd.model.getpos);

    offset = [0, 0, -0.06];

    tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trD.T' * transl(offset)';
    set(sponge,'Vertices',tranVertspo(:,1:3));
end

% animate Dobot & sponge moving
for j = 1:1:4
    qMoD1 = rd.model.getpos;
    qMoD2 = rd.model.ikcon(transSpoD2{j}, moveD{j});
    qMatrixD2 = jtraj(qMoD1,qMoD2,stepslow);

    for i = 1:1:stepslow
        rd.model.animate(qMatrixD2(i,:));

        % Animate sponge moving with arm
        trD = rd.model.fkine(rd.model.getpos);

        offset = [0, 0, -0.06];

        tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trD.T' * transl(offset)';
        set(sponge,'Vertices',tranVertspo(:,1:3));
    end
end

% solve IK to place sponge by Dobot
qPlD1 = rd.model.getpos;
qPlD2 = rd.model.ikcon(transSpoD3*rpy2tr(0,0,-pi/2), placeSpoDIG1);
qMatrixD3 = jtraj(qPlD1,qPlD2,stepslow);

% animate sponge placement by Dobot
for i = 1:1:stepslow
    rd.model.animate(qMatrixD3(i,:));

    % Animate sponge moving with arm
    trD = rd.model.fkine(rd.model.getpos);

    offset = [0, 0, -0.06];

    tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trD.T' * transl(offset)';
    set(sponge,'Vertices',tranVertspo(:,1:3));
end

% solve IK to home Dobot
qHoD1 = rd.model.getpos;
qHoD2 = rd.model.ikcon(transHoD1, homeDIG1);
qMatrixD3 = jtraj(qHoD1,qHoD2,stepslow);

% animate Dobot home
for i = 1:1:stepslow
    rd.model.animate(qMatrixD3(i,:));
end

%% animate TM12 again
% solve IK to pickup sponge by TM12
qPiU1 = rl.model.getpos;
qPiU2 = rl.model.ikcon(transSpoU1, pickSpoUIG1);
qMatrixU1 = jtraj(qPiU1,qPiU2,steps);

% animate TM12 pickup sponge
for i = 1:1:steps
    rl.model.animate(qMatrixU1(i,:));
end

% solve IK to place sponge by TM12
qPlU1 = rl.model.getpos;
qPlU2 = rl.model.ikcon(transSpoU4*rpy2tr(0,0,-pi/2), placeSpoUIG1);
qMatrixU2 = jtraj(qPlU1,qPlU2,steps);

% animate sponge placement by TM12
for i = 1:1:steps
    rl.model.animate(qMatrixU2(i,:));

    % Animate sponge moving with arm
    trU = rl.model.fkine(rl.model.getpos);

    % offset = [0, 0, -0.06];

    tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trU.T' * transl(offset)';
    set(sponge,'Vertices',tranVertspo(:,1:3));
end

% RMRC animate
stepR = 20;
deltaT = 0.05;

qU1 = [qMatrixU2(3,:)];

for j = 5:1:8-1
    tranSpoUplus = transSpoU3{(j+1)};
    moveSpoUIGplus = moveSpoUIG{(j+1)};
    qU2 = rl.model.ikcon(tranSpoUplus, moveSpoUIGplus);
    % qU2 = rl.model.ikcon(transSpoU3{2}, moveSpoUIG{2});
    qMatrixU3 = jtraj(qU1, qU2, stepR);

    x1 = transSpoU3{j}(1:3, 4);
    x2 = tranSpoUplus(1:3, 4);
    x = zeros(3, stepR);
    s = lspb(0, 1, stepR);
    for i = 1:stepR
        x(:, i) = x1*(1-s(i)) + s(i)*x2;
    end

    qMatrixU3 = nan(stepR, 6);
    qMatrixU3(1, :) = qU1;

    for i = 1:1:stepR-1
        xdot_linear = (x(:, i+1) - x(:, i)) / deltaT;
        xdot_angular = [0; 0; 0];
        xdot = [xdot_linear; xdot_angular];

        J = rl.model.jacob0(qMatrixU3(i, :));
        qdot = pinv(J)*xdot;
        qMatrixU3(i+1, :) = qMatrixU3(i, :) + deltaT*qdot';
    end

    for i = 1:1:stepR
        rl.model.animate(qMatrixU3(i,:));

        % Collision detection
        allTransforms = cell(1, rl.model.n);  % Pre-allocate a cell array for the transformations
        currentT = eye(4);  % Start with the identity matrix

        qT = qMatrixU3(i,:);

        linkT6 = rl.model.fkine(qT);

        capsuleLink6.Pose = linkT6.T;

        % show(capsuleLink6);

        % Animate sponge moving with arm
        trU = rl.model.fkine(rl.model.getpos);

        offset = [0, 0, -0.06];

        tranVertspo = [vertspo, ones(size(vertspo,1),1)] * trU.T' * transl(offset)';
        set(sponge,'Vertices',tranVertspo(:,1:3));
        % End of sponge animation

        % isColliding6 = checkCollision(capsuleLink6, box1);

        % if isColliding6
        %     error('Collision detected');
        % else
        %     disp('No collisions detected');
        % end
    end

    qU1 = qMatrixU3(end, :);
end

% solve IK to home TM12
qHoU1 = rl.model.getpos;
qHoU2 = homeUIG1;
qMatrixU4 = jtraj(qHoU1,qHoU2,steps);

% animate TM12 home
for i = 1:1:steps
    rl.model.animate(qMatrixU4(i,:));
end

%%
% rd.model.teach;
% rl.model.teach(qlinear);

