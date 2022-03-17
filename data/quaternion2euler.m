function [Euler] = quaternion2euler(Quat)

	% Extract the sin of the theta angle
	sth = max(min(2.0*(Quat(1)*Quat(3)-Quat(4)*Quat(2)),1.0),-1.0); % Note: If Quat is valid then this should only trim at most a few eps...

	% Calculate the required Euler angles
	Q3sq = Quat(3)*Quat(3);
	Euler = [atan2(2.0*(Quat(1)*Quat(4)+Quat(2)*Quat(3)),1.0-2.0*(Q3sq+Quat(4)*Quat(4))) asin(sth) atan2(2.0*(Quat(1)*Quat(2)+Quat(3)*Quat(4)),1.0-2.0*(Quat(2)*Quat(2)+Q3sq))];

end
