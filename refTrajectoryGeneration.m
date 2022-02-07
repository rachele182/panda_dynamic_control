function [ q, qdot, qddot, tt ] = refTrajectoryGeneration( tWaypoints, qWaypoints, tt )
% Generate reference trajectory
if nargout == 1
    q = pchip(tWaypoints,qWaypoints,tt); 
else
    ndof = size(qWaypoints,2);
    dt = 0.01;
    Q = [qWaypoints(1,:);  qWaypoints; qWaypoints(end,:)]';
    T = [0, tWaypoints+dt, tWaypoints(end)+2*dt] ;
    tt = [0, tt + dt, tt(end)+2*dt];

    pp_pos = spline(T,Q);
    pieces = pp_pos.pieces;

    coefs_vel = zeros(ndof*pieces,3); 
    coefs_acc = zeros(ndof*pieces,2);
    for k = 1:ndof
        for i = 1:pieces
            c = pp_pos.coefs((k-1)*pieces+i,:);
            coefs_vel((k-1)*pieces+i,:) = [3*c(1) 2*c(2) c(3)];
            coefs_acc((k-1)*pieces+i,:) = [6*c(1) 2*c(2)];
        end
    end
    
    pp_vel = pp_pos;
    pp_vel.order = 3;
    pp_vel.coefs = coefs_vel; 
    
    pp_acc = pp_pos;
    pp_acc.order = 2;
    pp_acc.coefs = coefs_acc;

    q = ppval(pp_pos, tt)';
    qdot = ppval(pp_vel, tt)';
    qddot = ppval(pp_acc, tt)';
end
end