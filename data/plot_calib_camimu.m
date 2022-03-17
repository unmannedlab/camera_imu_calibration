clear all;
clc;
data_ext_calib = csvread('camimu_calib_extrinsic.csv');
eul_xyz = data_ext_calib(:, 1:3)*180/pi;
t_xyz = data_ext_calib(:, 4:6);
sigma_rxryrz = data_ext_calib(:, 7:9)*180/pi;
sigma_txyz = data_ext_calib(:, 10:12);
%%
figure('Name','Calibration XYZ KF','NumberTitle','off');
subplot(311)
plot(t_xyz(:,1),  '.', 'LineWidth', 3);
%hold off;
hold on;
plot(t_xyz(:,1) + sigma_txyz(:,1), '--r', 'LineWidth', 3);
hold on;
plot(t_xyz(:,1) - sigma_txyz(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib X [m]','fontweight','bold','fontsize',16);
grid;
title('Calib X [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16)

subplot(312)
plot(t_xyz(:,2),  '.', 'LineWidth', 3);
%hold off;
hold on;
plot(t_xyz(:,2) + sigma_txyz(:,2), '--r', 'LineWidth', 3);
hold on;
plot(t_xyz(:,2) - sigma_txyz(:,2), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib Y [m]','fontweight','bold','fontsize',16);
grid;
title('Calib Y [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);

subplot(313)
plot(t_xyz(:,3),  '.', 'LineWidth', 3);
%hold off;
hold on;
plot(t_xyz(:,3) + sigma_txyz(:,3), '--r', 'LineWidth', 3);
hold on;
plot(t_xyz(:,3) - sigma_txyz(:,3), '--r', 'LineWidth', 3);
hold off;
ylabel('Calib Z [m]','fontweight','bold','fontsize',16);
grid;
title('Calib Z [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);


%%
eulerangleDegrees = eul_xyz;
eulerangleDegreesMinus = eulerangleDegrees - sigma_rxryrz;
eulerangleDegreesPlus = eulerangleDegrees + sigma_rxryrz;

%%
figure('Name','Rotation Calibration KF','NumberTitle','off');
subplot(311)
plot(eulerangleDegrees(:,1),  '.', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,1), '--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,1), '--r', 'LineWidth', 3);
hold off;
ylabel('Euler X','fontweight','bold','fontsize',16);
grid;
title('Euler X [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([170, 190]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(312)
plot(eulerangleDegrees(:,2), '.',  'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,2),'--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,2),'--r',  'LineWidth', 3);
hold off;
ylabel('Euler Y','fontweight','bold','fontsize',16);
grid;
title('Euler Y [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([-10, 10]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(313)
plot(eulerangleDegrees(:,3), '.',  'LineWidth', 3);
hold on;
plot(eulerangleDegreesMinus(:,3),'--r', 'LineWidth', 3);
hold on;
plot(eulerangleDegreesPlus(:,3),'--r',  'LineWidth', 3);
hold off;
ylabel('Euler Z','fontweight','bold','fontsize',16);
grid;
title('Euler Z [deg]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
% ylim([-10, 10]);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);


