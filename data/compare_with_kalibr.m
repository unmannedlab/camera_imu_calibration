clear;
I_T_C = importdata('I_T_C.txt');
I_T_C_kalibr = [-0.02715127 0.04190112 0.99875277 0.06453266;
                 0.99962776 0.00381096 0.02701517 0.1360674;
                -0.00267424 0.99911449 -0.04198899 -0.0734004; 
                          0 0           0         1];
T_err = I_T_C*inv(I_T_C_kalibr)
R_err = T_err(1:3, 1:3)
t_err = T_err(1:3, 4)
eul_err = rotm2eul(R_err)*180/pi
euler_IC = rotm2eul(I_T_C(1:3, 1:3))*180/pi
euler_IC_kalibr = rotm2eul(I_T_C_kalibr(1:3, 1:3))*180/pi