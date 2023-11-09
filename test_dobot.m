%% Before
%before running make sure to start roscore
%After that open new cmd terminal and nagvigate to cd ~/catkin_ws
%run source devel/setup.bash
%run roslaunch dobot_magician_driver dobot_magician.launch
%run rostopic list in another cmd terminal to check everything is running,
%you should see a list of dobot topics
%carefully run next step by step
%% Start
clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit;

%% Start Dobot ROS
dobot = DobotMagician();
%% Reinitilise Robot
dobot.InitaliseRobot();

%% home
end_effector_position = [0.2322,-0.0294, 0.0469];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
% close tool
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
%% Move from home to sponge
end_effector_position = [0.2469,-0.0050, 0.0163];
end_effector_rotation = [0.1056,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
% open tool
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Move gripper onto sponge
end_effector_position = [0.2469,-0.0019, -0.0087];
end_effector_rotation = [0.0142,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
%% close tool
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);

%% Raise Sponge into air
end_effector_position = [0.2743,0.0033, 0.1047];
end_effector_rotation = [0.0200,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
%% Move to point in between sink  to avoid limit error
end_effector_position = [0.1767,-0.2073,0.1071];
end_effector_rotation = [-0.8765 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
%% Move sponge to "sink"
end_effector_position = [-0.0166,-0.2303,0.0853];
end_effector_rotation = [-0.1745 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Move into in sink
end_effector_position = [-0.0218,-0.2347,0.0195];
end_effector_rotation = [-0.0235 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(2)

%dip

end_effector_position = [-0.0232,-0.2379,0.0295];
end_effector_rotation = [-0.0048 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)


%dip
end_effector_position = [-0.0218,-0.2347,0.0195];
end_effector_rotation = [-0.0235 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)

%dip
end_effector_position = [-0.0232,-0.2379,0.0295];
end_effector_rotation = [-0.0048 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)


%dip
end_effector_position = [-0.0218,-0.2347,0.0195];
end_effector_rotation = [-0.0235 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)

%dip
end_effector_position = [-0.0232,-0.2379,0.0295];
end_effector_rotation = [-0.0048 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)


%dip
end_effector_position = [-0.0218,-0.2347,0.0195];
end_effector_rotation = [-0.0235 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1)

%dip
end_effector_position = [-0.0232,-0.2379,0.0295];
end_effector_rotation = [-0.0048 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);


%% Move sponge into air
end_effector_position = [-0.0166,-0.2303,0.0853];
end_effector_rotation = [-0.1745 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% move to avoid imit erro
end_effector_position = [0.1767,-0.2073,0.1071];
end_effector_rotation = [-0.8765 0 0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Move sponge to TM12 pick up
end_effector_position = [0.2743,0.0033, 0.1047];
end_effector_rotation = [0.0200,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);


%% Lower sponge
end_effector_position = [0.2469,-0.0019, -0.0087];
end_effector_rotation = [0.0142,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Release Gripper
% open tool
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);


%% Move back to home position
end_effector_position = [0.2322,-0.0294, 0.0469];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);


pause
%% E-STop
clear register a
a = arduino('/dev/ttyACM0','Uno','Libraries','ShiftRegister');
configurePin(a,'D2','pullup');
time = 200;
for i = 0.1:0.05:1.0 
 disp('Dobot good');
 i
 joint_target = [0.0,i,0.3,0.0];
 dobot.PublishTargetJoint(joint_target)

 button_status = readDigitalPin(a, 'D2')      
 if button_status == 0
     disp('Push buttons pressed');
     dobot.EStopRobot();
     clear register a
     time=0;
     break;
 end
 pause(0.5);
end