%% generate trajectory task space

%% COMPARE (FREE MOTION)
%des trajectory
% [xd,dxd,ddxd] = gen_traj(x_in,time);
[xd,dxd,ddxd] = circ_traj(x_in,time); 
psi_ext = zeros(6,1);
%complian traj

[xc,dxc,ddxc] = adm_contr(xd,dxd,ddxd,psi_ext,time,x_in,dx_in,Md,Kd,Bd); 


%% PLOTS
%get position
pos_d = [zeros(size(time,2),4)];
pos_r = [zeros(size(time,2),4)];
y = [zeros(size(time,2),3)];
yr = [zeros(size(time,2),3)];
for j = 1:size(time,2)
    pos_d(j,:) = vec4(DQ(xd(j,:)).translation);
    pos_r(j,:) = vec4(DQ(xc(j,:)).translation);
    y(j,:) = [pos_d(j,2),pos_d(j,3),pos_d(j,4)];
    yr(j,:) = [pos_r(j,2),pos_r(j,3),pos_r(j,4)];
end

figure()
tiledlayout(3,1) 
nexttile
plot(time, pos_d(:,2), 'Linewidth',1.5, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,2), 'Linewidth',1.5, 'Color', 'b')
hold on
grid on
xlabel('time [s]')
ylabel('x [m]')
legend('des','comp')
nexttile
plot(time, pos_d(:,3), 'Linewidth',1.5, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,3), 'Linewidth',1.5, 'Color', 'b')
hold on
grid on
xlabel('time [s]')
ylabel('y [m]')
legend('des','comp')

nexttile
plot(time, pos_d(:,4), 'Linewidth',1.5, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,4), 'Linewidth',1.5, 'Color', 'b')
hold on
grid on
xlabel('time [s]')
ylabel('z [m]')
legend('des','comp')


