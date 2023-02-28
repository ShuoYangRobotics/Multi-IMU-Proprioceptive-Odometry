function quat = euler_to_quat(euler)
    % convert r p y euler angle to quat

    roll = euler(1);
    pitch = euler(2);
    yaw = euler(3);

    yaw = yaw / 2.0;
    pitch = pitch / 2.0;
    roll = roll / 2.0;

    cos_half_yaw = cos(yaw);
    sin_half_yaw = sin(yaw);
    cos_half_pitch = cos(pitch);
    sin_half_pitch = sin(pitch);
    cos_half_roll = cos(roll);
    sin_half_roll = sin(roll);
    quat = [
        cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw;
        sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw;
        cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw;
        cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw;];

end