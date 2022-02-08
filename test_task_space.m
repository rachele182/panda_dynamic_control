%% Test task space controller
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

%% neighbourhood waypoints 
q1 = q_in + [0;0;0;deg2rad(-5);0;0;0];
q2 = q_in + [0;deg2rad(-10);0;0;0;0;0];
q3 = q_in + [0;deg2rad(+10);0;0;0;0;0];
q4 = q_in + [0;deg2rad(-10);0;0;0;0;0];
q5 = q_in + [0;deg2rad(+10);0;0;0;0;0];

tWaypoints = [0,0.5,1,1.5,2];
qWaypoints = [q1,q2,q3,q4,q5]';

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
   
    % Setting to synchronous mode
    %---------------------------------------
    sim.simxSynchronous(clientID,true)   
    sim.simxSynchronousTrigger(clientID)
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_oneshot)


    %  start our simulation in lockstep 
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    %---------------------------------------
    
    % Getting joint-positions
    %---------------------------------------
    for j=1:7
        [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_blocking);
    end      
    qm = double([qmread])';
    qdot_m = double([qdot])';

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
        
        %---------------------------------------    
        for j=1:7
            [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_blocking);
        end      
        qmOld = qm;
        % Current joint configuration 
        disp('Measured joint position')
        qm = double([qmread])' %current joint postions
        qdot_m = double([qdot])' %curent joint velocities
        
        % Current EE configuration
        disp('Current EE configuration')
        xdq = fep.fkm(qm); %current ee-pose
        T1 = DQuaternionToMatrix(xdq.q')
        x = T1(1:3,4); %current ee_position
        
        % Current joint derivative (Euler 1st order derivative)
        qm_dot_calculated = (qm-qmOld)/cdt
        qm_dot = qdot_m;

        %Get jacobians
        Jp = fep.pose_jacobian(qm);
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
        %% Impedance control
        % Using the dynamic model
        O = zeros(1,6);
        I = eye(3);
        A = [O; I zeros(3,1) zeros(3,1) zeros(3,1); ...
             O;zeros(3,1) zeros(3,1)  zeros(3,1) I;];
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm);
        tauf = get_FrictionTorque(qm_dot);
        %set gains
        Km = eye(8)*20; 
        Dm = eye(8)*5; 
        % define error 
        e = haminus8(DQ(C8*vec8(cdq)))*vec8(cdq - xdq);
        e_dot = haminus8(DQ(C8*Jp*dq'))*vec8(cdq - xdq) + haminus8(DQ(C8*vec8(cdq)))*Jp*(dq' - qm_dot);
        % define cl dynamics
        y =  Dm*e_dot + Km*e;
        ddxr = Jp_dot*dq' + Jp*ddq';
        J = geomJ(fep,qm);
        Jg2 = A*J;
        % control input task space
        ax = haminus8(DQ(ddxr)')*vec8(cdq - xdq) + 2*haminus8(DQ(C8*Jp*dq'))*Jp*(dq' - qm_dot)+ haminus8(DQ(C8*vec8(cdq)))*(ddxr - Jp_dot*qm_dot) + y; 
        J_inv = pinv(haminus8(DQ(C8*vec8(cdq)))*Jp);
        %control input joint space
%       aq = J_inv*ax; 
        aq = Jg2'*ax;
        %fb linearization
        tau = M*aq + c + g ;
        
        %store for later
        tau_send = tau;
        sres.tau_send(:,i) = tau_send;
        
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
        