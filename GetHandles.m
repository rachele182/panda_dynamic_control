function handles = GetHandles(clientID, vrep)
%   GETHANDLES 
%
%   Get handles of all important scene components.
%
%   INPUTS:       CLIENTID -> Handle for V-rep client;
%                     VREP -> Handle for V-rep connection;
%
%
%   OUTPUTS:       HANDLES -> Handles of all components.
%

%Frames
[~,world_frame_handle]=vrep.simxGetObjectHandle(clientID,'Floor',vrep.simx_opmode_oneshot_wait); % World frame handle
[~,world_frame_handle2]=vrep.simxGetObjectHandle(clientID,'RezisableFloor_5_25',vrep.simx_opmode_oneshot_wait); % World frame handle

%Objects
[~,object_handle] = vrep.simxGetObjectHandle(clientID,'Cup',vrep.simx_opmode_oneshot_wait); % Center of mass reference handle
%Joints franka 1
[~,joints_handles(1)] = vrep.simxGetObjectHandle(clientID,'Franka_joint1',vrep.simx_opmode_oneshot_wait); % 2nd joint handle
[~,joints_handles(2)] = vrep.simxGetObjectHandle(clientID,'Franka_joint2',vrep.simx_opmode_oneshot_wait); % 1st joint handle
[~,joints_handles(3)] = vrep.simxGetObjectHandle(clientID,'Franka_joint3',vrep.simx_opmode_oneshot_wait); % 3rd joint handle 
[~,joints_handles(4)] = vrep.simxGetObjectHandle(clientID,'Franka_joint4',vrep.simx_opmode_oneshot_wait); % 4th joint handle
[~,joints_handles(5)] = vrep.simxGetObjectHandle(clientID,'Franka_joint5',vrep.simx_opmode_oneshot_wait); % 5th joint handle
[~,joints_handles(6)] = vrep.simxGetObjectHandle(clientID,'Franka_joint6',vrep.simx_opmode_oneshot_wait); % 6th joint handle
[~,joints_handles(7)] = vrep.simxGetObjectHandle(clientID,'Franka_joint7',vrep.simx_opmode_oneshot_wait); % 7th joint handle

%Effector franka 1
[~,EE_handle]=vrep.simxGetObjectHandle(clientID,'BarrettHand',vrep.simx_opmode_oneshot_wait); % End effector handle
[~,attach_point_handle] = vrep.simxGetObjectHandle(clientID,'Franka_connection',vrep.simx_opmode_oneshot_wait);

%OPEN THE CHANNELS TO READ INFO LATER
for i=1:7
    vrep.simxGetObjectPosition(clientID,joints_handles(i),-1,vrep.simx_opmode_streaming);
    vrep.simxGetObjectOrientation(clientID,joints_handles(i),-1,vrep.simx_opmode_streaming);
    vrep.simxGetJointPosition(clientID,joints_handles(i),vrep.simx_opmode_streaming);
end


%PREPARING OUTPUTS
handles.worldFrame = world_frame_handle;
handles.worldFrame2 = world_frame_handle2;
handles.joints = joints_handles;
handles.object = object_handle; 
handles.effector = EE_handle;
handles.attach_point = attach_point_handle;

end
