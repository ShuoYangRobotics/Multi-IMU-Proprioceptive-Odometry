
addpath('auto_func')


%% put some necessary variables in param struct
param.rho_opt_size = 1;
param.rho_fix_size = 4;
param.rho_opt_str = '$l_c$';

param.num_leg = 4;
param.leg_name = ['FL','FR','RL', 'RR'];
param.all_leg = [1,2,3,4];
% assume all legs are active 
param.active_leg = [1,1,1,1];
param.ox = [0.1805,0.1805,-0.1805,-0.1805];
param.oy = [0.047,-0.047,0.047,-0.047];
param.d = [0.0838,-0.0838,0.0838,-0.0838];
% param.d = [0.09,-0.09,0.09,-0.09];
param.lt = 0.20;
param.lc = 0.20;

param.rho_opt_true = zeros(param.rho_opt_size,4);
param.rho_opt_true(:,1) = [param.lc];
param.rho_opt_true(:,2) = [param.lc];
param.rho_opt_true(:,3) = [param.lc];
param.rho_opt_true(:,4) = [param.lc];

param.rho_opt_init = zeros(param.rho_opt_size,4);
param.rho_opt_init(:,1) = [param.lc]+0.1*randn(1,1);
param.rho_opt_init(:,2) = [param.lc]+0.1*randn(1,1);
param.rho_opt_init(:,3) = [param.lc]+0.1*randn(1,1);
param.rho_opt_init(:,4) = [param.lc]+0.1*randn(1,1);

param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt];
