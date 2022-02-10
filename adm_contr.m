function [xc,dxc,ddxc] = adm_contr(xd,dxd,ddxd,psi_ext,time,x_in,dx_in,Md,Kd,Bd)

%% Description: admittance controller to enforce an apparent impedance behaviour at task-space
%%inputs: xd,dxd,ddxd = desired reference trajectory
%         psi_ext = external wrench on EE
%         time = simulation time
%         Md,Kd,Bd = impedance matrices 
%outpus:  xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames

%initialize
xc = [zeros(size(time,2),8)];
dxc = [zeros(size(time,2),8)];
ddxc = [zeros(size(time,2),8)];
cdt = time(2) - time(1); 
xr_in = vec8(x_in);
dxr_in = dx_in;

l= 1; 
for l = 1:size(time,2)
    x_hat = vec8(DQ(xr_in)'*DQ(xd(l,:))); %pose displacement between desired and compliant frame
    y_hat = vec6(log(DQ(x_hat))); %log mapping of pose displacement
    dx_hat = vec8(DQ(dxr_in)'*DQ(xd(l,:)) + DQ(xr_in)'*DQ(dxd(l,:)));
    Q8 = getQ8(DQ(x_hat));
    dy_hat = pinv(Q8)*(dx_hat);
    
    %Mapping external wrench to be consisten with DQ log definition
    Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
        zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
    G = getG(DQ(x_hat));
    Glog = Ibar*G*Q8;
    flog = (Glog)'*(psi_ext);
    
    %admittance equation
    ddy_hat = inv(Md)*(-Bd*dy_hat-Kd*y_hat-flog);
    Q8_dot = getQ8_dot(DQ(x_hat),DQ(dx_hat));
    
    ddx_hat = Q8*ddy_hat + Q8_dot*dy_hat;
    dx_hat =  ddx_hat*cdt + dx_hat;
    x_hat = dx_hat*cdt + x_hat;
    
    %retrieve compliant trajectory
    xr = hamiplus8(DQ(xd(l,:)))*DQ.C8*(x_hat);
    dxr = hamiplus8(DQ(dxd(l,:)))*DQ.C8*(x_hat) + hamiplus8(DQ(xd(l,:)))*DQ.C8*(dx_hat);
    ddxr = hamiplus8(DQ(ddxd(l,:)))*DQ.C8*(x_hat) + 2*hamiplus8(DQ(dxd(l,:)))*DQ.C8*(dx_hat) + hamiplus8(DQ(xd(l,:)))*DQ.C8*ddx_hat;
    
    %store output
    xc(l,:) = xr;
    dxc(l,:) = dxr;
    ddxc(l,:) = ddxr;
    l = l+1;
    
    %update values
    xr_in = xr;
    dxr_in = dxr;
    
end

end



