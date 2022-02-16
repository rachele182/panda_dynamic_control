%% TEST ADMITTANCE 

%des trajectory
% [xd,dxd,ddxd] = gen_traj(x_in,time);
[xd,dxd,ddxd,int_data] = int_traj(x_in,time); 


fi_data = zeros(size(time,2),1);
wrench_ext_data = zeros(size(time,2),6); 

j = 1;
for j = 1:size(time,2)
    if j~=1
        fi = wrench_ext_data(j-1,3)';
        t_curr = time(j);
        t_prec = time(j-1);
    else
        fi = 0;
        t_curr = time(j);
        t_prec = 0; 
    end
    wrench_ext = ext_forces(int_data(j,:),xd(j,:),fi,t_curr,t_prec);
    wrench_ext_data(j,:) = wrench_ext;
    j = j+1;
end

figure()
plot(time,wrench_ext_data(:,3)); 


xc_data = zeros(size(time,2),8);
dxc_data = zeros(size(time,2),8);
ddxc_data = zeros(size(time,2),8);

yr_data = zeros(size(time,2),6);
dyr_data =  zeros(size(time,2),6);


j = 1;
for j = 1:size(time,2)
    if j~=1
        xr = xc_data(j-1,:)';
        yr_in = yr_data(j-1,:)';
        dyr_in = dyr_data(j-1,:)';
    else
        xr = vec8(x_in);
        e_in = vec8(DQ(xr)'*DQ(xd(1,:)));
        yr_in = vec6(log(DQ(e_in)));
        dyr_in = zeros(6,1);
    end

    %compliant traj
    [xc,dxc,ddxc,yr,dyr] = adm_contr_online(xd(j,:),dxd(j,:),ddxd(j,:),wrench_ext_data(j,:),xr,yr_in,dyr_in,Md,Kd,Bd,time);

    xc_data(j,:) = xc; 
    dxc_data(j,:) = dxc;
    ddxc_data(j,:) = ddxc;
    yr_data(j,:) = yr; 
    dyr_data(j,:) = dyr; 

    j = j+1;
end


% %% PLOTS
% %get position
pos_d = [zeros(size(time,2),4)];
pos_r = [zeros(size(time,2),4)];
y = [zeros(size(time,2),3)];
yr = [zeros(size(time,2),3)];
for j = 1:size(time,2)
    pos_d(j,:) = vec4(DQ(xd(j,:)).translation);
    pos_r(j,:) = vec4(DQ(xc_data(j,:)).translation);
    y(j,:) = [pos_d(j,2),pos_d(j,3),pos_d(j,4)];
    yr(j,:) = [pos_r(j,2),pos_r(j,3),pos_r(j,4)];
end

figure()
tiledlayout(3,1) 
nexttile
plot(time, pos_d(:,2), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,2), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('x [m]')
legend('des','comp')
nexttile
plot(time, pos_d(:,3), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,3), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('y [m]')
legend('des','comp')

nexttile
plot(time, pos_d(:,4), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, pos_r(:,4), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('z [m]')
legend('des','comp')



