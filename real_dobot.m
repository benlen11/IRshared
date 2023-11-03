clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit;

%% Start Dobot ROS
dobot = DobotMagician();

%% Test Motion

%% Find current position
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
pause(2); %Allow some time for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z]
% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];
% Convert from quaternion to euler
currentEndEffectorQuat = transpose(currentEndEffectorQuat);
currentEndEffectorPos = quat2eul(currentEndEffectorQuat)
%% Publish custom joint target
joint_target = [0.5,0.5,0.5,0.5];
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
end_effector_position = [0.0,0,0.05];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Turn on tool
% close tool
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
%%
% open tool
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Turn off tool
onOff = 0;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Test ESTOP 
%% Send this first
for i = 0.1:0.05:1.0
    joint_target = [0.0,i,0.3,0.0];
    dobot.PublishTargetJoint(joint_target);
    pause(0.1);
end

%% When the robot is in motion, send this
dobot.EStopRobot();

%% Reinitilise Robot
dobot.InitaliseRobot();

%% Set Rail status to true
dobot.SetRobotOnRail(true);

%% Reinitialise robot. It should perform homing with the linear rail
dobot.InitaliseRobot();

%% Move the rail to the position of 0.1
dobot.MoveRailToPosition(0.1);

%% Set Rail status to false
dobot.SetRobotOnRail(false);

%% Reinitialise robot. It should not perform homing with the linear rail
dobot.InitaliseRobot();

%% Test IO
%% Get current IO status of all io pins on the robot
[ioMux, ioData] = dobot.GetCurrentIOStatus();

%% Set a particular pin a particular IO output
address = 1;
ioMux = 1;
data = 1;
dobot.SetIOData(address,ioMux, data);

%% Set particular pin a particular IO input
address = 1;
ioMux = 3;
data = 0;
dobot.SetIOData(address,ioMux,data);

%% Set a velocity to the conveyor belt
enableConveyor = true;
conveyorVelocity = 15000; % Note: this is ticks per second
dobot.SetConveyorBeltVelocity(enableConveyor,conveyorVelocity);

%% Turn off conveyor belt
enableConveyor = false;
conveyorVelocity = 0; % Note: this is ticks per second
dobot.SetConveyorBeltVelocity(enableConveyor,conveyorVelocity);
