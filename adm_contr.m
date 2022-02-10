function [xc,dxc,ddxc] = adm_contr(xd,dxd,ddxd,psi_ext,time)

%% Description: admittance controller to enforce an apparent impedance behaviour at task-space
%%inputs: xd,dxd,ddxd = desired reference trajectory
%         psi_ext = external wrench on EE
%         time = simulation time
%outpus:  xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames

%initialize
xc = [zeros(size(time,2),8)];
dxc = [zeros(size(time,2),8)];
ddxc = [zeros(size(time,2),8)];

xr_in = x_in;
dxr_in = dx_in;

for i = 1:size(time,2)
    x_hat = DQ(xr_in)*DQ(xd(i,:)); %pose displacement between desired and compliant frame
    x_hat_old = x_hat;
    y_hat = log(x_hat); %log mapping of pose displacement
    dx_hat = DQ(dxr_in)*DQ(xd(i,:)) + DQ(xr_in)*DQ(xd(i,:));
    dx_hat_old = dx_hat;
    Q8 = getQ8(x_hat);
    dy_hat = pinv(Q8)*dx_hat;
    
    %Mapping external wrench to be consisten with DQ log definition
    Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
        zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
    G = getG(x_hat);
    Glog = Ibar*G*Q8;
    flog = (Glog)'*(psi_ext);
    
    %admittance equation
    ddy_hat = inv(Md)*(-Bd*dy_hat-Kd*y_hat-flog);
    Q8_dot = getQ8_dot(x_hat,dx_hat);
    
    ddx_hat = Q8*ddy_hat + Q8_dot*dy_hat;
    dx_hat =  ddx_hat*cdt + dx_hat_old;
    x_hat = dx_hat*cdt + x_hat_old;
    
    %retrieve compliant trajectory
    xr = hamiplus8(DQ(xd(i,:)))*C8*x_hat;
    dxr = hamiplus8(DQ(dxd(i,:)))*C8*x_hat + hamiplus8(DQ(xd(i,:)))*C8*dx_hat;
    ddxr = hamiplus8(DQ(ddxd(i,:)))*C8*x_hat + 2*hamiplus8(DQ(dxd(i,:)))*C8*dx_hat + hamiplus8(DQ(xd(i,:)))*C8*ddx_hat;
    
    %store output
    xc(i,:) = xr;
    dxc(i,:) = dxr;
    ddxc(i,:) = ddxr;
    i = i+1;
    
    %update values
    xr_in = xr;
    dxr_in = dxr;
    
end

end



