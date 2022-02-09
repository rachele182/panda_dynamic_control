function pose_joints = GetPoseJoints(clientID, vrep, world_frame_handle, joints_handles)
%   GETPOSEJOINTS 
%
%   Get pose of all joints.
%
%   INPUTS:           CLIENTID -> Handle for V-rep client;
%                         VREP -> Handle for V-rep connection;
%           WORLD_FRAME_HANDLE -> Handle for world frame;
%               JOINTS_HANDLES -> Handles for all joints (nx1 vector).
%
%
%   OUTPUTS:    POSE_JOINTS -> Pose of all joints (nx1 vector).
%

n = length(joints_handles);
for i=1:n
    pose_joints(i) = GetPose(clientID, vrep, world_frame_handle, joints_handles(i));
end

end