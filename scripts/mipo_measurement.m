function meas_residual = mipo_measurement(x, wk, phik, dphik, yawk, foot_gyrok, param)

% this measurement function calculates mipo measurement

% state x 
%   - position          (1:3)
%   - velocity          (4:6)
%   - euler angle       (7:9)
%   - foot1 pos         (10:12)
%   - foot1 vel         (13:15)
%   - foot2 pos         (16:18)
%   - foot2 vel         (19:21)
%   - foot3 pos         (22:24)
%   - foot3 vel         (25:27)
%   - foot4 pos         (28:30)
%   - foot4 vel         (31:33)
%   - body acc bias     (34:36)
%   - body gyro bias    (37:39)
%   - foot1 acc bias    (40:42)
%   - foot2 acc bias    (43:45)
%   - foot3 acc bias    (46:48)
%   - foot4 acc bias    (49:57)
%   - time  tk          (52)

    if ~iscolumn(x)
        x = x';
    end


    pos = x(1:3);
    vel = x(4:6);
    euler = x(7:9);
    R_er = euler_to_rot(euler);
    foot_pos = [x(10:12);x(16:18);x(22:24);x(28:30)];
    foot_vel = [x(13:15);x(19:21);x(25:27);x(31:33)];
    bg = x(37:39);

    meas_per_leg = param.mipo_meas_per_leg;
    meas_residual = zeros(param.mipo_meas_size,1,class(x));
    for i = 1:param.num_leg
        angle = phik((i-1)*3+1:(i-1)*3+3);
        av = dphik((i-1)*3+1:(i-1)*3+3);
        p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        leg_v = (J_rf*av+skew(wk-bg)*p_rf);



        meas_residual((i-1)*meas_per_leg+1:(i-1)*meas_per_leg+3) = ...
            p_rf - R_er'*(foot_pos((i-1)*3+1:(i-1)*3+3) - pos);
        meas_residual((i-1)*meas_per_leg+4:(i-1)*meas_per_leg+6) = ...
            foot_vel((i-1)*3+1:(i-1)*3+3) - (vel + R_er*leg_v);

        % use foot gyro to calculate a velocity
        foot_w_robot = foot_gyrok((i-1)*3+1:(i-1)*3+3);
        foot_w_world = R_er*foot_w_robot;
        p_rf_world = R_er*p_rf;   % the vector pointing from body to foot fl in world frame

        foot_support_vec = -p_rf_world/norm(p_rf_world)*0.05;
        foot_vel_world = cross(foot_w_world, foot_support_vec);

        if (param.mipo_use_foot_ang_contact_model == 1)
            meas_residual((i-1)*meas_per_leg+7:(i-1)*meas_per_leg+9) = ...
                foot_vel((i-1)*3+1:(i-1)*3+3)-foot_vel_world;
        else 
            meas_residual((i-1)*meas_per_leg+7:(i-1)*meas_per_leg+9) = ...
                foot_vel((i-1)*3+1:(i-1)*3+3);
        end

        % foot height should be 0 
        meas_residual((i-1)*meas_per_leg+10) = foot_pos((i-1)*3+3);   % assume ground always has 0 height 
    end

    meas_residual(end) = yawk - euler(3);



end