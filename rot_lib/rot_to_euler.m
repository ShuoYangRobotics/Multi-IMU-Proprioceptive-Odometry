function euler = rot_to_euler(rot)

	cos_pitch = sqrt(rot(1,1)*rot(1,1) + rot(2,1) * rot(2,1));

	yaw   = atan2(rot(2,1), rot(1,1));
	pitch = atan2(-rot(3,1), cos_pitch);
	roll  = atan2(rot(3,2), rot(3,3));

    euler = [roll;pitch;yaw];

end