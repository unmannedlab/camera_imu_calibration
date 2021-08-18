clear
I_T_C = importdata('I_T_C.txt');
I_R_C = I_T_C(1:3, 1:3);
I_t_C = I_T_C(1:3, 4)
eul = rotm2eul(I_R_C)*180/pi