function transform_kalibr_to_cincalib(str)
  I_T_C_kalibr = importdata(str);;
  T_offset =[0,  0, 1, 0;
            -1,  0, 0, 0;
             0, -1, 0, 0;
             0,  0, 0, 1];                          
  I_T_C_kalibr_transformed =  I_T_C_kalibr*inv(T_offset);
  csvwrite(str, I_T_C_kalibr_transformed);
endfunction                