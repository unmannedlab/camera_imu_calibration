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

