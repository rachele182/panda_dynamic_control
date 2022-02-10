%% Main;
clear all;
clear classes;
warning off;
clc;

%% Addpath 
% addpath /home/geriatronics/github/panda_dynamic_control/functions
% addpath /home/geriatronics/github/panda_dynamic_control/Vrep_utils

addpath /home/rachele/github/panda_dynamic_control/functions
addpath /home/rachele/github/panda_dynamic_control/Vrep_utils

disp('Loading parameters..')

%% Joint limits
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Constants
C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
threshold = 1e-12; 
cdt = 0.01; %sampling time
time = 0:cdt:10; %simulation time

%% Admittance controller gains
I = eye(6);
Md = 1.5*I;  %desired mass matrix
Kd = 1000*I; %desired stiffness matrix 
Bd = 2*sqrt(1000*1.5)*I*0.5;  %desired damping matrix

%% Build robot
FEp_DH_theta = [0, 0, 0, 0, 0, 0, 0];
FEp_DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
FEp_DH_a = [0, 0, 0.0825, -0.0825, 0, 0.088 0.0003];
FEp_DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2 0];
FEp_DH_matrix = [FEp_DH_theta; FEp_DH_d; FEp_DH_a; FEp_DH_alpha]; 
franka = DQ_SerialManipulator(FEp_DH_matrix,'standard');

%%Initial conditions
q_in = [1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %initial joint angles
q1 = q_in  +[0;0;0;deg2rad(-5);0;0;0];
x_in = franka.fkm(q_in); 
x_in_2 = franka.fkm(q1);
dx_in = zeros(8,1);
