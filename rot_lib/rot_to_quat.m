function quat = rot_to_quat(rot)

	trace = rot(1,1) + rot(2,2) + rot(3,3);
	S = 0.0;

	if (trace > 0)
		S = sqrt(trace + 1.0) * 2.0;								
		w = 0.25 * S;
		x = (rot(3,2) - rot(2,3)) / S;
		y = (rot(1,3) - rot(3,1)) / S;
		z = (rot(2,1) - rot(1,2)) / S;
    elseif ((rot(1,1) > rot(2,2)) && (rot(1,1) > rot(3,3)))
		S = sqrt(1.0 + rot(1,1) - rot(2,2) - rot(3,3)) * 2.0;	
		w = (rot(3,2) - rot(2,3)) / S;
		x = 0.25 * S;
		y = (rot(1,2) + rot(2,1)) / S;
		z = (rot(1,3) + rot(3,1)) / S;
    elseif (rot(2,2) > rot(3,3))
		S = sqrt(1.0 + rot(2,2) - rot(1,1) - rot(3,3)) * 2.0;	
		w = (rot(1,3) - rot(3,1)) / S;
		x = (rot(1,2) + rot(2,1)) / S;
		y = 0.25 * S;
		z = (rot(2,3) + rot(3,2)) / S;
    else
		S = sqrt(1.0 + rot(3,3) - rot(1,1) - rot(2,2)) * 2.0;	
		w = (rot(2,1) - rot(1,2)) / S;
		x = (rot(1,3) + rot(3,1)) / S;
		y = (rot(2,3) + rot(3,2)) / S;
		z = 0.25 * S;
    end
    quat = [w;x;y;z];
end