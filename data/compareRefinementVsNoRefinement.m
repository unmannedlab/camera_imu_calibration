clear all;
%close all;
clc;
data_wrefinement = csvread('camerapose_trajectory_wrefinement.csv');
data_worefinement = csvread('camerapose_trajectory_worefinement.csv');

xyz_wrefinement = data_wrefinement(:, 5:7);
xyz_worefinement = data_worefinement(:, 5:7);

%%
figure('Name', 'Camera Translation X, Y, Z','NumberTitle','off')
subplot(311)
plot(xyz_wrefinement(:,1), 'LineWidth', 3);
hold on;
plot(xyz_worefinement(:,1), 'LineWidth', 3);
hold off;
ylabel('X','fontweight','bold','fontsize',16);
grid;
title('X [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('w refinement', 'wo refinement');
set(gca,'FontSize', 24);

subplot(312)
plot(xyz_wrefinement(:,2), 'LineWidth', 3);
hold on;
plot(xyz_worefinement(:,2), 'LineWidth', 3);
hold off;
ylabel('Y','fontweight','bold','fontsize',16);
grid;
title('Y [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('w refinement', 'wo refinement');
set(gca,'FontSize', 24);

subplot(313)
plot(xyz_wrefinement(:,3), 'LineWidth', 3);
hold on;
plot(xyz_worefinement(:,3), 'LineWidth', 3);
hold off;
ylabel('Z','fontweight','bold','fontsize',16);
grid;
title('Z [m]','fontweight','bold','fontsize',16);
xlabel('Time [s]','fontweight','bold','fontsize',16);
legend('w refinement', 'wo refinement');
set(gca,'FontSize', 24);