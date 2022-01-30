#data_cincalib_process = csvread('reprojection_error.csv'); 
data_cincalib_validation = csvread('reprojection_error_validation.csv'); 
figure(1)
#plot(data_cincalib_process, 'linewidth', 5);
#hold on;
plot(data_cincalib_validation, 'linewidth', 3);
hold off;
grid;
title('CIN Calib Process vs Validation');
#legend('Cin Calib Process', 'Cin Calib Validation');