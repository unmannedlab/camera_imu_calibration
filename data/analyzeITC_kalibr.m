T_ic =  [-0.02193031 -0.02335384  0.9994867   0.08700428;
 0.99975431  0.00270997  0.0219995   0.13405822;
-0.00322235  0.99972359  0.02328867 -0.06947327;
 0.          0.          0.          1.        ]
R_ic = T_ic(1:3, 1:3);
t_ic = T_ic(1:3, 4)
eul = rotm2eul(R_ic)*180/pi              