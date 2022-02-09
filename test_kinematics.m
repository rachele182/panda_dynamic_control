%% test kinematics
clear;
clc;
close all;

%%Addpath 
include_namespace_dq;

%%disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

i=1;

vi = DQ_VrepInterface;

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics
% fep.base_frame = (fep_vreprobot.vrep_interface.get_object_pose(fep_vreprobot.base_frame_name));


if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Vrep disconneted')
end
    handles = get_joint_handles(sim,clientID);   
    joint_handles = handles.armJoints;
    utils = GetHandles(clientID, sim);   
    pose_joints = GetPoseJoints(clientID, sim, utils.worldFrame, joint_handles);
%     fep.set_base_frame(pose_joints(1));


fep  = fep_vreprobot.kinematics();

    pause(0.3);
    
    % get initial state of the robot
    qstr = '[ ';
    qdotstr = '[ ';
    
    for j=1:7
        [res,q(j)] = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_buffer);
        [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_buffer);
        qstr = [qstr,num2str(q(j)),' '];
        qdotstr = [qdotstr,num2str(qdot(j)),' '];
    end
    
    qstr = [qstr,']']
    qdotstr = [qdotstr,']']
    disp('Initial Joint positions: ')
    disp(qstr)
    disp('Initial Joint Velocities: ')
    disp(qdotstr)
    
    
%     q_in2 = [1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %initial joint conf of franka scene
  q_in2 = [0 0 0 -1.5708 0 1.5708 0]';  %initial joint conf of free motion (to test flange orientation easily)
    pos_in = fep.fkm(q_in2).translation; %initial EE position
    p0 = vec4(pos_in)
    phi = fep.fkm(q_in2).rotation_angle
    n = fep.fkm(q_in2).rotation_axis
    quat = vec4(fep.fkm(q_in2).P);
   