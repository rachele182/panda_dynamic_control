function [xc,dxc,ddxc] = adm_contr_online(xd,dxd,ddxd,psi_ext,xr,dxr,Md,Kd,Bd)

%% Description: admittance controller to enforce an apparent impedance behaviour at task-space
%%inputs: xd,dxd,ddxd = desired reference trajectory
%         psi_ext = external wrench on EE (with respect to compliant reference frame)
%         time = simulation time
%         Md,Kd,Bd = impedance matrices
%outpus:  xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames


%Initialize
xr_in = xr;
e_in = vec8(DQ(xr_in)'*DQ(xd));
yr_in = vec6(log(DQ(e_in)));
dyr_in = dxr;

x_hat = vec8(DQ(xr_in)'*DQ(xd)); %pose displacement between desired and compliant frame
y_hat = yr_in; %log mapping of pose displacement
Q8 = getQ8(DQ(x_hat));
dy_hat = dyr_in;

%Mapping external wrench to be consisten with DQ log definition
Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
    zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
G = getG(DQ(x_hat));
Glog = Ibar*G*Q8;
flog = (Glog)'*(psi_ext(l,:)');

%admittance equation
ddy_hat = inv(Md)*(-Bd*dy_hat-Kd*y_hat-flog);
dy_hat  = ddy_hat*cdt + dy_hat;
y_hat = dy_hat*cdt + y_hat;

x_hat = vec8(exp(DQ(y_hat)));
Q8 = getQ8(DQ(x_hat));
dx_hat = Q8*dy_hat;
Q8_dot = getQ8_dot(DQ(x_hat),DQ(dx_hat));
ddx_hat = Q8*ddy_hat + Q8_dot*dy_hat;

%retrieve compliant trajectory
xc = hamiplus8(DQ(xd))*DQ.C8*(x_hat);
dxc = hamiplus8(DQ(dxd))*DQ.C8*(x_hat) + hamiplus8(DQ(xd))*DQ.C8*(dx_hat);
ddxc = hamiplus8(DQ(ddxd))*DQ.C8*(x_hat) + 2*hamiplus8(DQ(dxd))*DQ.C8*(dx_hat) + hamiplus8(DQ(xd))*DQ.C8*ddx_hat;


end



