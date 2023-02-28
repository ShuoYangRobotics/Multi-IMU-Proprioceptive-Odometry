function meas_residual = sipo_measurement(x, wk, phik, dphik, yawk, param)

% this measurement function calculates four position measurements from all
% legs, and then just list them 

    if ~iscolumn(x)
        x = x';
    end

    pos = x(1:3);
    state_v = x(4:6);
    euler = x(7:9);
    R_er = euler_to_rot(euler);
    foot_pos = x(10:21);
    bg = x(25:27);
%     bg = zeros(3,1);

    meas_per_leg = param.sipo_meas_per_leg;
    meas_residual = zeros(param.sipo_meas_size,1,class(x));
    for i = 1:param.num_leg
        angle = phik((i-1)*3+1:(i-1)*3+3);
        av = dphik((i-1)*3+1:(i-1)*3+3);
        p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        leg_v = (J_rf*av+skew(wk-bg)*p_rf);


        meas_residual((i-1)*meas_per_leg+1:(i-1)*meas_per_leg+3) = ...
            p_rf - R_er'*(foot_pos((i-1)*3+1:(i-1)*3+3) - pos);
        meas_residual((i-1)*meas_per_leg+4:(i-1)*meas_per_leg+6) = ...
            state_v + R_er*leg_v;

        % foot height should be 0 
        meas_residual((i-1)*meas_per_leg+7) = foot_pos((i-1)*3+3);   % assume ground always has 0 height 
    end
    meas_residual(end) = yawk - euler(3);



end