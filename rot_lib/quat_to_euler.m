function euler = quat_to_euler(quat)

    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

	cos_pitch_cos_yaw  = 1.0- 2.0 * (y*y + z*z);   
	cos_pitch_sin_yaw  =      + 2.0 * (x*y + w*z);   
	sin_pitch	         =      - 2.0 * (x*z - w*y);  
	cos_pitch			 = 0.0;
	sin_roll_cos_pitch =      + 2.0 * (y*z + w*x);   
	cos_roll_cos_pitch = 1.0 - 2.0 * (x*x + y*y);   


	cos_pitch = sqrt(cos_pitch_cos_yaw*cos_pitch_cos_yaw + cos_pitch_sin_yaw*cos_pitch_sin_yaw);

	yaw   = atan2(cos_pitch_sin_yaw, cos_pitch_cos_yaw);
    if abs(sin_pitch) >= 1
	    pitch = sign(sin_pitch)*pi/2;
    else
	    pitch = asin(sin_pitch);
    end
	roll  = atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

    euler = zeros(3,1);
    euler(1) = roll;
    euler(2) = pitch;
    euler(3) = yaw;

end