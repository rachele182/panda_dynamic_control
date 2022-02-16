function wrench_ext = ext_forces(int,x,fi,t_curr,t_prec)
%% Description: model external forces acting on the end effectors
%% Assumptions: elastic reaction of the environment; negligible external torques
% Output: wrench_ext = vector 6x1 representing the external wrench on EE (world frame)
% Inputs: x = current EE pose;
%         pe = contact position;

k_table = 1000*5; %environment stiffness
pc = 0.35; %contact position (z axis)
cdt = 0.01; %sampling time

x_pos = vec4(DQ(x).translation); %EE position
z = [x_pos(2); x_pos(3); x_pos(4)];
% F_ext = 0;

    if z(3) < pc 
        F_ext = -k_table*(z(3) - pc); %elastic reaction
%         fz = -k_table*(z(3) - pc); %elastic reaction
%         F_ext = fi + ((fz-fi)/cdt)*(t_curr - t_prec);
%     else
%         if int == 1

%     elseif z(3) > pc+0.002
%         if(fi~=0)
%             F_ext = 0.5*fi;
%         else
%             F_ext = 0; 
%         end
    else
        F_ext = 0;
        
    end
% else
%    F_ext = 0;
% end

wrench_ext = [0;0;F_ext;0;0;0];

end