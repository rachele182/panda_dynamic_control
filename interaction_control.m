%% Test simple interaction task

%% TEST MOTION CONTROL TASK SPACE TRAJECTORY
%%Addpath 
include_namespace_dq;

%% intiialize
wrench_ext = [zeros(size(time,2),6)]; %external wrench on EE (world_frame)
psi_ext = [zeros(size(time,2),6)];

%% Desired trajectory
cdt = 0.01; %sampling time (10ms)

[xd1, dxd1, ddxd1] = int_traj(x_in,time); %minimum jerk trajectory (desired)

%% Connect to VREP

disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);
i=1;
vi = DQ_VrepInterface;

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics

if (clientID>-1)
    disp('Connected to remote API server');
    
    handles = get_joint_handles(sim,clientID);
    joint_handles = handles.armJoints;
    fep  = fep_vreprobot.kinematics(); 
    
    for j=1:7
        [res,q(j)] = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_buffer);
        [res,qdot(j)] = sim.simxGetObjectFloatParameter(clientID,joint_handles(j),2012,sim.simx_opmode_buffer);
    end
   
    
    %% Setting to synchronous mode
    %---------------------------------------
    sim.simxSynchronous(clientID,true)   
    sim.simxSynchronousTrigger(clientID)
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_oneshot)
    %start simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    %---------------------------------------
    
    %% Get joint positions
    %---------------------------------------
    for j=1:7
        [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
        [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
    end      
    qm = double([qmread])';
    
    % Saving data to analyze later
    sres.xd = [];  sres.xd_dot = [];  sres.xd_ddot = [];
    sres.x = []; sres.xref = []; sres.f_ext = []; 
    %---------------------------------------    
    % time
    inittime = sim.simxGetLastCmdTime(clientID);
    
%% Control loop   
    while sim.simxGetConnectionId(clientID)~=-1
        
        if i>size(time,2)
            break
        end
        
        % Getting joint-position
        %---------------------------------------    
        for j=1:7
            [~,tauOrig(j)] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            [~,qmread(j)]  = sim.simxGetJointPosition(clientID,joint_handles(j),sim.simx_opmode_blocking);
        end      
        qmOld = qm;
        % Current joint configuration 
        qm = double([qmread])';
        
        % Current EE configuration
        x = vec8(fep.fkm(qm)); 
        
        % Model forces
        x_pos = vec4(DQ(x).translation); %crrent ee position
        r0 = DQ(x).P; %initial EE rotation
        z = [x_pos(2); x_pos(3); x_pos(4)];
        

        if z(3) < z_table
            f_ext = -k_table*(z(3) - z_table);
        else
            f_ext = 0;
        end

        wrench_ext(i,:) = [0;0;f_ext;0;0;0];
        psi_ext(i,:) = vec6((r0)'*DQ(wrench_ext(i,:))*(r0)); %external wrench (compliant frame)


        % Pose Jacobian
        Jp = fep.pose_jacobian(qm);
        
        % Geometric Jacobian
        J = geomJ(fep,qm);
        Jg = [J(4:6,:);J(1:3,:)]; %[translation-rotation]
        
        % Current joint derivative (Euler 1st order derivative)
        qm_dot = (qm-qmOld)/cdt; %computed as vrep function 
        
        %Current 1st-time derivative of EE pose
        dx = Jp*qm_dot;
        
        % Pose Jacobian first-time derivative 
        Jp_dot = fep.pose_jacobian_derivative(qm,qm_dot);
        %---------------------------------------    
        

        [xd,dxd,ddxd] = adm_contr(xd1,dxd1,ddxd1,psi_ext(i,:),time,x_in,dx_in,Md1,Kd1,Bd1); %(compliant trajectory) 

        % Compliant trajectory position,velocity acceleration
        xd_des = xd(i,:)';
        dxd_des = dxd(i,:)';
        ddxd_des = ddxd(i,:)'; 
        
        %Desired trajectory
        xd1_str = xd1(i,:);
        dx1_str = dxd1(i,:);
        ddxd1_str = ddxd1(i,:);
        
        %Ext force
        fext = psi_ext(i,1:3)';
       
        % Printing the time step of the simulation and the error
        % -----------------------
       
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - error:',num2str(norm(xd_des-x))])
        disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - pos error:',num2str(norm(vec4(DQ(xd_des).translation-DQ(x).translation)))])
        
        % Saving data to analyze later
        % -----------------------
        
        sres.xd(:,i) = vec4(DQ(xd_des).translation);  sres.xd_dot(:,i) = dxd_des;  sres.xd_ddot(:,i) = ddxd_des;
        sres.x(:,i) = vec4(DQ(x).translation); 
        sres.xref(:,i) = vec4(DQ(xd1_str).translation);
        sres.fext(:,i) = fext; 
      
        % -----------------------        
        
        % Using the dynamic model
        g = get_GravityVector(qm);
        c = get_CoriolisVector(qm,qm_dot);
        M = get_MassMatrix(qm); 
        tauf = get_FrictionTorque(qm_dot);                

%%       Task-space inverse dynamics with fb linearization
         kp = 1000;
         kd = 100;
         ki = 500; %integral gain 
         
         %% Define error (task-space)
         e = xd_des - x;
         de = dxd_des - dx;
         ei = de*cdt + e;
         y = pinv(Jp)*(ddxd_des - Jp_dot*qm_dot  + kp*eye(8)*e + kd*eye(8)*de + 0*ki*eye(8)*ei);
         tau = M*y + c + g; 
         
         N = haminus8(DQ(xd_des))*DQ.C8*Jp;
         robustpseudoinverse = N'*pinv(N*N' + 0.1*eye(8));
         
         %%%%%%%% null space control %%%%%%%%%
         P = eye(7) - pinv(N)*N;
         D_joints = eye(7)*2;
         tau_null = P*(-D_joints*qm_dot);
         tau_ext = Jg'*wrench_ext(i,:)';
         tau = tau + tau_null + 0*tau_ext;
         
         %Sent torque commands
         tau_send = tau;
         sres.tau_send(:,i) = tau_send;
         
        %---------------------------------------
             
        %% Send torques to vrep
        for j=1:7
            if tau(j)<0
                set_vel = -99999;
            else
                set_vel = 99999;
            end
            % blocking mode
            %---------------------------------         
            sim.simxSetJointTargetVelocity(clientID,joint_handles(j),set_vel,sim.simx_opmode_blocking);            
            sim.simxSetJointForce(clientID,joint_handles(j),abs(tau(j)),sim.simx_opmode_blocking);
            [~,tau_read] = sim.simxGetJointForce(clientID,joint_handles(j),sim.simx_opmode_blocking);
            tau_read_data(:,j) = tau_read;   
        end
        
        sres.tau_read(i,:) = tau_read_data';
        %---------------------------------
        sim.simxSynchronousTrigger(clientID);
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

%% PLOTS if wanted

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

%%Plot ee-position
figure();
plot(tt,sres.xd(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.xref(2,:),'b','LineWidth',2)
legend('xc','x','xd')
figure();
plot(tt,sres.xd(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(3,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(3,:),'b','LineWidth',2)
legend('yc','y','yd')
figure()
plot(tt,sres.xd(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres.x(4,:),'c','LineWidth',2);
hold on,grid on
plot(tt,sres.xref(4,:),'b','LineWidth',2)
legend('zc','z','zd')

%%Plot ext force
figure()
plot(tt,sres.f_ext(1,:),'LineWidth',2);
hold on, grid on
plot(tt,sres.f_ext(2,:),'LineWidth',2);
hold on,grid on
plot(tt,sres.f_ext(3,:),'LineWidth',2);
