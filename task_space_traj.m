
function [t,pos_i,pos_f,tf,r0] = task_space_traj(x_in,time)
%retrieve intial EE pose

p0 = vec4(x_in.translation);
r0 = vec4(P(x_in));
pos_i = [p0(2);p0(3);p0(4)];

for i = 1:size(time,2)
    if (time(i)>=0 && time(i)<0.5) %go down
        pos_f = pos_i + [0;0;-0.3];
        tf = 0.5;
        t = time(i);
    elseif (time(i)>=0.5 && time(i)<0.6) %pause
        pos_i = [p0(2);p0(3);p0(4)-0.3];
        pos_f = pos_i;
        tf = 0.1;
        t = time(i) - 0.5;
    elseif (time(i)>=0.6 && time(i)<1.1) %go +x
        pos_i = [p0(2);p0(3);p0(4)-0.3];
        pos_f = pos_i + [0.1;0;0];
        tf = 0.5;
        t = time(i) - 0.6;
    elseif (time(i)>=1.1 && time(i)<1.2) %pause
        pos_i = [p0(2)+0.1;p0(3);p0(4)-0.3];
        pos_f = pos_i;
        tf = 0.1;
        t = time(i) - 1.1;
    elseif (time(i)>=1.2 && time(i)<1.7) %go up and backwards
        pos_i = [p0(2)+0.1;p0(3);p0(4)-0.3];
        pos_f = pos_i + [-0.1;0;+0.3];
        tf = 0.5;
        t = time(i)-1.2;
    else
        pos_i = [p0(2);p0(3);p0(4)];
        pos_f = pos_i;
        tf = 1000;
        t = time(i) - 1.7;
    end
end
end

function [xd,dxd,ddxd] = traj(x_in,time)
%% description: compute minimum jerk trajectory
[t,pos_i,pos_f,tf,r0] =  task_space_traj(x_in,time);
%inputs: t = time
%        pos_i = initial position
%        pos_f = final position
%        tf = desired time frame for trajectory
%outpus: xd,dxd,ddxd desired position,velocity and acceleration
xd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
dxd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
ddxd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));

end



function [x_des,dx_des,ddx_des] = gen(x_in,time)
%% Compute trajectory within dual quaternion domain
%dual quaternion trajectory
[xd,dxd,ddxd] = traj(x_in,time);
x_des = vec8(DQ(r0) + 0.5*DQ.E*(DQ(xd)*DQ(r0)));
dx_des =  vec8(0.5*DQ.E*(DQ(dxd)*DQ(r0)));
ddx_des = vec8(0.5*DQ.E*(DQ(ddxd)*DQ(r0)));
end 




