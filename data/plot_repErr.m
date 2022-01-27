data_cincalib_process = csvread('reprojection_error_cincalib_process.csv'); 
data_cincalib = csvread('reprojection_error_cincalib.csv');
data_kalibr = csvread('reprojection_error_kalibr.csv');

figure(1)
plot(data_cincalib_process, 'linewidth', 5);
grid;
title('CIN Calib Process');

figure(2)
plot(data_cincalib_process, 'linewidth', 5);
hold on;
plot(data_cincalib, 'linewidth', 5);
hold off;
grid;
legend('cincalib-process', 'cincalib');
title('CIN Calib Process vs CIN Calib Verification');

figure(3)
plot(data_cincalib_process, 'linewidth', 5);
hold on;
plot(data_kalibr, 'linewidth', 5);
hold on;
plot(data_cincalib, 'linewidth', 5);
hold off;
grid;
legend('cincalib-process', 'kalibr', 'cincalib');
title('CIN Calib Process vs Kalibr and CIN Calib Verification');

figure(4)
plot(data_kalibr, 'linewidth', 5);
hold on;
plot(data_cincalib, 'linewidth', 5);
hold off;
grid;
legend('cincalib-process', 'kalibr', 'cincalib');
title('Kalibr and CIN Calib Verification');