%% generate trajectory task space
%%Addpath 
clear;
close;

include_namespace_dq;
vi = DQ_VrepInterface;

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);
fep  = fep_vreprobot.kinematics(); 

%% Load robot

ts = 0.01; % sampling time
time = 0:0.01:2; %simulation time
q_in = [0 0 0 -1.5708 0 1.5708 0]';
x_in = fep.fkm(q_in);

[xd,dxd,ddxd] = gen(x_in,time);

psi_ext = zeros(size(time,2),6);
[xc,dxc,ddxc] = adm_contr(xd,dxd,ddxd,psi_ext,time); 
    
