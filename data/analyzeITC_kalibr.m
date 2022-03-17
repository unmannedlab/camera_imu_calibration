T_ic =  [-0.02193031 -0.02335384  0.9994867   0.08700428;
 0.99975431  0.00270997  0.0219995   0.13405822;
-0.00322235  0.99972359  0.02328867 -0.06947327;
 0.          0.          0.          1.        ]
R_ic = T_ic(1:3, 1:3);
t_ic = T_ic(1:3, 4)

Rotmat = R_ic;
% Extract the sin of the theta angle
sth = max(min(-Rotmat(3,1),1.0),-1.0); % Note: If Rotmat is valid then this should only trim at most a few eps...

% Calculate the required Euler angles
Euler = [atan2(Rotmat(2,1),Rotmat(1,1)) asin(sth) atan2(Rotmat(3,2),Rotmat(3,3))]*180/pi
  