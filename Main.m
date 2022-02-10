%% Main;
clear all;
clear classes;
warning off;
clc;

%% Addpath 
addpath /home/geriatronics/github/panda_dynamic_control/functions
addpath /home/geriatronics/github/panda_dynamic_control/Vrep_utils
disp('Loading parameters..')

%% Joint limits
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Constants
C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
threshold = 1e-12; 
cdt = 0.01; %sampling time

%% Admittance controller gains
I = eye(6);
Md = 1.5*I;  %desired mass matrix
Kd = 1000*I; %desired stiffness matrix 
Bd = 2*sqrt(1000*1.5)*I;  %desired damping matrix

%% Build robot
f_d = [0.333 0 0.316 0 0.384 0 0 0.107];
f_thetas = [0 0 0 0 0 0 0 0];
f_a = [0 0 0 0.0825 -0.0825 0 0.088 0];
f_alphas = [0 -pi/2 pi/2 pi/2 -pi/2 pi/2 pi/2 0];
franka_dh_matrix = [f_thetas; 
                    f_d; 
                    f_a;
                    f_alphas];
franka = DQ_SerialManipulator(franka_dh_matrix,'modified'); 

%% Initial conditions
% q_in
% x_in  dx_in %intial EE DQ pose
