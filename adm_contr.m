function [xc,dxc,ddxc] = adm_contr(xd,dxd,ddxd,psi_ext,time,x_in,dx_in,Md,Kd,Bd)

%% Description: admittance controller to enforce an apparent impedance behaviour at task-space
%%inputs: xd,dxd,ddxd = desired reference trajectory
%         psi_ext = external wrench on EE (with respect to compliant reference frame)
%         time = simulation time
%         Md,Kd,Bd = impedance matrices 
%outpus:  xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames

%prepare outputs
xc = [zeros(size(time,2),8)];
dxc = [zeros(size(time,2),8)];
ddxc = [zeros(size(time,2),8)];
cdt = time(2) - time(1); %sampling time

%Initialize
xr_in = vec8(x_in);
dxr_in = dx_in;
e_in = vec8(DQ(xr_in)'*DQ(xd(1,:)));
yr_in = vec6(log(DQ(e_in)));
dyr_in = zeros(6,1); 

l= 1; 

for l = 1:size(time,2)
    x_hat = vec8(DQ(xr_in)'*DQ(xd(l,:))); %pose displacement between desired and compliant frame
    y_hat = yr_in; %log mapping of pose displacement
    Q8 = getQ8(DQ(x_hat));
    dy_hat = dyr_in;
    
    %Mapping external wrench to be consisten with DQ log definition
    Ibar = [zeros(3,1), eye(3), zeros(3,1), zeros(3,3);...
        zeros(3,1), zeros(3,3), zeros(3,1), eye(3)];
    G = getG(DQ(x_hat));
    Glog = Ibar*G*Q8;
    flog = (Glog)'*(psi_ext);
    
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
    xr = hamiplus8(DQ(xd(l,:)))*DQ.C8*(x_hat);
    dxr = hamiplus8(DQ(dxd(l,:)))*DQ.C8*(x_hat) + hamiplus8(DQ(xd(l,:)))*DQ.C8*(dx_hat);
    ddxr = hamiplus8(DQ(ddxd(l,:)))*DQ.C8*(x_hat) + 2*hamiplus8(DQ(dxd(l,:)))*DQ.C8*(dx_hat) + hamiplus8(DQ(xd(l,:)))*DQ.C8*ddx_hat;
    
    %store output
    xc(l,:) = xr;
    dxc(l,:) = dxr;
    ddxc(l,:) = ddxr;
    
    %update values
    xr_in = xr;
    dxr_in = dxr;
    yr_in = y_hat; 
    dyr_in = dy_hat; 
    
    l = l+1;
       
end

end



