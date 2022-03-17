addpath ("RotationsLib")
data_camera_traj = csvread('camerapose_trajectory.csv');
data_camera_traj_imu_smoothed = csvread('camimucalib_trajectory.csv');

txyz_camera_traj = data_camera_traj(:, 5:7);
quat_camera_traj = data_camera_traj(:, 1:4);

txyz_camera_traj_imu_smoothed = data_camera_traj_imu_smoothed(:, 5:7);
quat_camera_traj_imu_smoothed = data_camera_traj_imu_smoothed(:, 1:4);

figure(1)
subplot(311)
plot(txyz_camera_traj(:, 1));
hold on;
plot(txyz_camera_traj_imu_smoothed(:, 1));
hold off;
grid;
subplot(312)
plot(txyz_camera_traj(:, 2));
hold on;
plot(txyz_camera_traj_imu_smoothed(:, 2));
hold off;
grid;
subplot(313)
plot(txyz_camera_traj(:, 3));
hold on;
plot(txyz_camera_traj_imu_smoothed(:, 3));
hold off;
grid;

euler_camera_traj = [];
for i=1:length(quat_camera_traj)
  quaternion = [quat_camera_traj(i,4), quat_camera_traj(i,1), 
                quat_camera_traj(i,2), quat_camera_traj(i,3)];
  euler_camera_traj = [euler_camera_traj; EulerFromQuat(quaternion)*180/pi];
end

euler_camera_traj_imu_smoothed = [];
for i=1:length(quat_camera_traj_imu_smoothed)
  quaternion = [quat_camera_traj_imu_smoothed(i,4), quat_camera_traj_imu_smoothed(i,1), 
                quat_camera_traj_imu_smoothed(i,2), quat_camera_traj_imu_smoothed(i,3)];
  euler_camera_traj_imu_smoothed = [euler_camera_traj_imu_smoothed; EulerFromQuat(quaternion)*180/pi];
end

figure(2)
subplot(311)
plot(euler_camera_traj(:, 1));
hold on;
plot(euler_camera_traj_imu_smoothed(:, 1));
hold off;
grid;
subplot(312)
plot(euler_camera_traj(:, 2));
hold on;
plot(euler_camera_traj_imu_smoothed(:, 2));
hold off;
grid;
subplot(313)
plot(euler_camera_traj(:, 3));
hold on;
plot(euler_camera_traj_imu_smoothed(:, 3));
hold off;
grid;