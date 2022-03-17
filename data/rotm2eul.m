function [Euler] = rotm2eul(Rotmat)

	% Extract the sin of the theta angle
	sth = max(min(-Rotmat(3,1),1.0),-1.0); % Note: If Rotmat is valid then this should only trim at most a few eps...

	% Calculate the required Euler angles
	Euler = [atan2(Rotmat(2,1),Rotmat(1,1)) asin(sth) atan2(Rotmat(3,2),Rotmat(3,3))];

end
