clear all;
%close all;
clc;
data = csvread('camimucalib_trajectory.csv');
quat = data(:, 1:4);
xyz = data(:, 5:7);
%%
figure('Name','IMU Trajectory 3D XYZ KF','NumberTitle','off');
plot3(xyz(1,1), xyz(1,2), xyz(1,3), 'ro','MarkerSize',12,'MarkerFaceColor',[1 .6 .6]);
hold on;
plot3(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'b', 'LineWidth', 1.5);
hold off;
grid;
axis equal;
xlabel('X [m]', 'fontweight', 'bold', 'fontsize', 16);
ylabel('Y [m]', 'fontweight', 'bold', 'fontsize', 16);
zlabel('Z [m]', 'fontweight', 'bold', 'fontsize', 16);
title('Trajectory of the Lidar-IMU system in IMU Frame', 'fontweight', 'bold', 'fontsize', 16);
legend('Start', 'Estimated IMU Trajectory', 'fontweight', 'bold',' fontsize', 16);
set(gca, 'FontSize', 20);

%%
figure('Name', 'IMU Translation X, Y, Z','NumberTitle','off')
subplot(311)
plot(xyz(:,1), '.', 'LineWidth', 3);
hold off;
ylabel('X','fontweight','bold','fontsize',16);
grid;
title('X [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(312)
plot(xyz(:,2), '.', 'LineWidth', 3);
hold off;
ylabel('Y','fontweight','bold','fontsize',16);
grid;
title('Y [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(313)
plot(xyz(:,3), '.', 'LineWidth', 3);
hold off;
ylabel('Z','fontweight','bold','fontsize',16);
grid;
title('Z [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

figure('Name', 'IMU Translation X, Y, Z','NumberTitle','off')
subplot(311)
plot(xyz(:,1), '.', 'LineWidth', 3);
hold off;
ylabel('X','fontweight','bold','fontsize',16);
grid;
title('X [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(312)
plot(xyz(:,2), '.', 'LineWidth', 3);
hold off;
ylabel('Y','fontweight','bold','fontsize',16);
grid;
title('Y [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

subplot(313)
plot(xyz(:,3), '.', 'LineWidth', 3);
hold off;
ylabel('Z','fontweight','bold','fontsize',16);
grid;
title('Z [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('Estimate', '1 \sigma bound');
set(gca,'FontSize', 24);

##%%
##eulerangleDegrees = [];
##for i=1:length(quat)
##    quat_i = quaternion(quat(i, 4), quat(i, 1), quat(i, 2), quat(i, 3));
##    eulerAngles = eulerd(quat_i, 'XYZ', 'frame');
##    euler_x = wrapTo360(eulerAngles(:, 1));
##    euler_y = eulerAngles(:, 2);
##    euler_z = eulerAngles(:, 3);
##    eulerangleDegrees = [eulerangleDegrees; [euler_x, euler_y, euler_z]];
##end
##
##figure('Name', 'IMU Euler X, Y, Z Angles','NumberTitle','off')
##subplot(311)
##plot(eulerangleDegrees(:,1), '.', 'LineWidth', 3);
##hold off;
##ylabel('Euler X','fontweight','bold','fontsize',16);
##grid;
##title('Euler X [deg]','fontweight','bold','fontsize',16);
##xlabel('Time [s]','fontweight','bold','fontsize',16);
##legend('Estimate', '1 \sigma bound');
##set(gca,'FontSize', 24);
##
##subplot(312)
##plot(eulerangleDegrees(:,2), '.', 'LineWidth', 3);
##hold off;
##ylabel('Euler Y','fontweight','bold','fontsize',16);
##grid;
##title('Euler Y [deg]','fontweight','bold','fontsize',16);
##xlabel('Time [s]','fontweight','bold','fontsize',16);
##legend('Estimate', '1 \sigma bound');
##set(gca,'FontSize', 24);
##
##subplot(313)
##plot(eulerangleDegrees(:,3), '.', 'LineWidth', 3);
##hold off;
##ylabel('Euler Z','fontweight','bold','fontsize',16);
##grid;
##title('Euler Z [deg]','fontweight','bold','fontsize',16);
##xlabel('Time [s]','fontweight','bold','fontsize',16);
##legend('Estimate', '1 \sigma bound');
##set(gca,'FontSize', 24);
