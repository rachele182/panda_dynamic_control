%% Test torque control vrep (joint space controller)
clear;
clc;
close all;

%%Addpath 
include_namespace_dq;

%% Compute desired joint space trajectory
q_in = [ 1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %initial joint angles

%% Generate desired joint_space trajectory
cdt = 0.010; %sampling time
tt = 0:cdt:2; %simulation time

%% neighbourhood waypoints (converges fast with joint space controller and kp = 1000, kd = 50);
q1 = q_in + [0;0;0;deg2rad(-5);0;0;0];
q2 = q_in + [0;deg2rad(-10);0;0;0;0;0];
q3 = q_in + [0;deg2rad(+10);0;0;0;0;0];
q4 = q_in + [0;deg2rad(-10);0;0;0;0;0];
q5 = q_in + [0;deg2rad(+10);0;0;0;0;0];

tWaypoints = [0,0.5,1,1.5,2];
qWaypoints = [q1,q2,q3,q4,q5]';

%% Circular trajectory
%load test_free_motion_jerk_traj.mat
% q1 = out.q.Data;
% tWaypoints = [0,0.5,1,1.5,2];
% qWaypoints = [q1(1,:);q1(51,:);q1(101,:);q1(151,:);q1(201,:)];


%% Generate trajectory joint space

[qDesired, qdotDesired, qddotDesired, tt] = refTrajectoryGeneration(tWaypoints, qWaypoints, tt);

%% Connect to vrep

disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

i=1;

vi = DQ_VrepInterface;

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics
fep  = fep_vreprobot.kinematics();

if (clientID>-1)
    disp('Connected to remote API server');
    
    handles = get_joint_handles(sim,clientID);
    joint_handles = handles.armJoints;
    
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
    
    qstr = [qstr,']'];
    qdotstr = [qdotstr,']'];
    disp('Initial Joint positions: ');
    disp(qstr);
    disp('Initial Joint Velocities: ');
    disp(qdotstr);
    
    
    % Setting to synchronous mode. This way vrep dynamics is only
    % executed once you tell it to do so. The problem with asynchronous (which
    % indeed can be more realistic) is that delay on computing the target
    % joints (to sent to vrep) may cause the robot to be in a completely
    % different configuration (due to the delay). This is specially true when
    % using matlab. 
    %---------------------------------------
    sim.simxSynchronous(clientID,true)   
    sim.simxSynchronousTrigger(clientID)
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_oneshot)


    %  start our simulation in lockstep 
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    %---------------------------------------
    
    % Getting joint-position (you cannot assume the desired q is the
    % real one and because it is not the real one, the dynamic configuration is
    % different).
    %---------------------------------------
    for j=1:7
        [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
    end      
    qm = double([qmread])';

    % Saving data to analyze later
    sres.qm = [];  sres.qm_dot = []; 
    sres.qd = [];  sres.qd_dot = [];  sres.qd_ddot = [];
    sres.T = [];
    %---------------------------------------    
    
    % time
    inittime = sim.simxGetLastCmdTime(clientID);
    
%% Control loop   
    while sim.simxGetConnectionId(clientID)~=-1
        
        if i>size(qDesired,1)
            break
        end
        
        % Getting joint-position (you need to do this in every step to get
        % the right dynamics (otherwise you will be getting the desired dynamics
        % which very likely is not the true one).  
        %---------------------------------------    
        for j=1:7
            [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
        end      
        qmOld = qm;
        % Current joint configuration 
        disp('Measured joint position')
        qm = double([qmread])'
        
        % Current EE configuration
        disp('Current EE configuration')
        xdq = fep.fkm(qm);
        T1 = DQuaternionToMatrix(xdq.q')
        x = T1(1:3,4); %current ee_position
        
        % Pose Jacobian
        Jp = fep.pose_jacobian(qm);
        
        
        % Geometric Jacobian
        J = geomJ(fep,qm);
        
        % Translation Jacobian
        J_t = J(4:6,:);
        
        % Current joint derivative (Euler 1st order derivative)
        qm_dot = (qm-qmOld)/cdt; 
        Jp_dot = fep.pose_jacobian_derivative(qm,qm_dot);
        %---------------------------------------    

        % Desired joint positions
        q = qDesired(i,:);
        
        % Desired Cartesian positions
        cdq = fep.fkm(q);
%         xd = cdq.q;
        T2 = DQuaternionToMatrix(cdq.q');
        xd = T2(1:3,4);
        
        % Printing the time step of the simulation and the error
        % from the desired to actual joint configurations
        % -----------------------
%         disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm( q'-qm ))])  
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm(vec8(cdq-xdq)))])
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - pos error:',num2str(norm(vec4(translation(normalize(cdq-xdq)))))])
        % Vector with both joints: desired vs real (simulated)
        disp('Vector with both joints: desired and real (simulated)')
        [q; double([qmread])]
        % -----------------------

        
        % Desired joint velocities
        dq = qdotDesired(i,:);
        
        % Desired joint accelerations
        ddq = qddotDesired(i,:);
        
        % Saving data to analyze later
        % -----------------------
        sres.qm(:,i) = qm;  sres.qm_dot(:,i) = qm_dot;  
        sres.qd(:,i) = q';  sres.qd_dot(:,i) = dq';  sres.qd_ddot(:,i) = ddq'; 
        sres.T(:,:,i) = T1;
        
        % -----------------------        
        
        % Using the dynamic model
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm);
        
        % The vrep model does not account for friction torques. Including
        % them leads to bad behaviour. 
        tauf = get_FrictionTorque(qm_dot);                
                

        % Compute motor torques (the open loop motion did not work and this
        % may be due to small differences in dynamics. 
        %---------------------------------------
        % Derivative gain (I tested quickly and the limit was 1 with 10ms)
        % More than this causes vrep to behave weirdly (not an issue with
        % the control but with the bullet dynamics engine).
        kd = (1/cdt);
%         kd = (0.8/cdt);

        % Proportinal gain (The limited I got was 20 with initial error
        % close to zero, but 10 if the error is not really close). If the
        % error is even larger, perhaps we need to reduce that as well.
        % This again is not a real issue (well, it can be if the torque is
        % too large) but rather a issue with the dynamics engine.
%         kp = (2/cdt);  % Takes longer to converge but smaller overshoot
%         kp = (5/cdt);   % Converges faster but larger overshoot
        kp = (10/cdt);   % Converges faster but larger overshoot

        % Stiffness matrix
        K = eye(8)*1000;
        
        % Damping matrix
%         D = diag([100, 100, 100]);
         D = eye(8)*5;
%          D = eye(7)*5;
%       
        
%        Simple PD Control Law in the joint space
%          tau = M*(ddq' + kd*(dq'-qm_dot) + kp*(q'-qm)) +c + g;
%        PD control in task space + g compensation 
         ki = 500; 
         e = vec8(cdq - xdq);
         de = Jp*(dq'-qm_dot);
         ei = de*cdt + e;
         y = pinv(Jp)*(Jp_dot*(dq'-qm_dot) + Jp*dq' + kp*eye(8)*e + kd*eye(8)*de + ki*eye(8)*ei);
         tau = M*y + c + g; 
%          tau = pinv(Jp)*(K * vec8((cdq - xdq)) - D*Jp*(qm_dot)) + c + g + M*ddq';
          

        % Control law is very simple (basically the desired acceleration is
        % cancelled while the velocity and position errors are controlled
        % (that is why we are getting the error of those values).
%         tau = M*(ddq' + kd*(dq'-qm_dot) + kp*(q'-qm)) +c + g;
         tau_send = tau;
         sres.tau_send(:,i) = tau_send;
         
        %---------------------------------------


%         tau_send = abs(tau);
%         tau_send = max(100,tau_send);
        
        
        % In synchronous mode this is not necessary
%         sim.simxPauseCommunication(clientID, 1);        
        % Send torque to vrep
        for j=1:7
            if tau(j)<0
                set_vel = -99999;
            else
                set_vel = 9999;
            end
            % blocking mode
            %---------------------------------         
            sim.simxSetJointTargetVelocity(clientID,joint_handles(j),set_vel,sim.simx_opmode_blocking);            
            sim.simxSetJointForce(clientID,joint_handles(j),abs(tau(j)),sim.simx_opmode_blocking);
            [~,tau_read] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            tau_read_data(:,j) = tau_read;
%             sres.tau_read(:,j) = tau_read_data;

            %---------------------------------
            

        end
        sres.tau_read(i,:) = tau_read_data';
        % In synchronous mode this is not necessary        
%         sim.simxPauseCommunication(clientID, 0);
        % Move vrep simulation one step up (pause to allow vrep
        % computations to work properly (just a safe measure, perhaps we do
        % need it)
        %---------------------------------
        sim.simxSynchronousTrigger(clientID);
        pause(0.002)
        %---------------------------------
        
        i = i+1;        
    end
   
    % Now close the connection to V-REP:
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

%% PLots if wanted
figure(); 
plot(tt,sres.qd_dot(1,:),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(1,:),'m','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(2,:),'b','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,sres.qm_dot(3,:),'g','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(4,:),'k','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(5,:),'r','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,sres.qm_dot(6,:),'c','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 
hold on, grid on
plot(tt,sres.qd_dot(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm_dot(7,:),'y','LineWidth',2);
legend('qdotDesired','qdotMeasured'); 

figure(); 
plot(tt,sres.qd(1,:),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(1,:),'m','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(2,:),'b','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(3,:),'g--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(3,:),'g','LineWidth',2);
hold on, grid on
legend('qDesired','qMeasured'); 
plot(tt,sres.qd(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(4,:),'k','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(5,:),'r','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(6,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(6,:),'c','LineWidth',2);
legend('qDesired','qMeasured'); 
hold on, grid on
plot(tt,sres.qd(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.qm(7,:),'y','LineWidth',2);
legend('qDesired','qMeasured'); 

figure(); 
plot(tt,sres.tau_read(:,1),'m--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_send(1,:),'m','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(2,:),'b--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(3,:),'g--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(4,:),'k--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(5,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(6,:),'c--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.tau_send(7,:),'y--','LineWidth',3); 
hold on, grid on
plot(tt,sres.tau_read(:,7),'y','LineWidth',2);
legend('tsend','tread'); 

