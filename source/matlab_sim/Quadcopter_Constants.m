% Mass moments of inertia about body frame's respective axes
% Mass moments of inertia about body frame's respective axes
Ixx = 0.0294838; 
Iyy = 0.0294838;
Izz = 0.058967624;

% Mass of quadroter
m = 0.914;

% Width and thickness (height) of quadcopter square approximation
w = 0.1872;
h = 0.00635;

% Moment arm
l = 0.254;

% Rotor intertia
Jr = (6e-5)*1.358;

% Aerodynamic force and moment constants
kf = 4.198756e-6;
km = 4.74157e-9;

% Motor circuit resistance and torque constants
Rmot = 0.6;
Kmot = 5.2;

% Aerodynamic translation and rotation coefficients
kt = (0.1)*1.358;
kr = (0.1)*1.358;
% These matrices can be used to test multiple
% drag coefficients simultaneously
% kt = blkdiag(0.1,0,0.15);
% kr = blkdiag(0.1,0,0.15);

% Gravity (down is positive in our equations)
g = 9.81;

pi = 3.14159265359;

% PID_Parameters
kd_phi = 1;
ki_phi = 1;
kp_phi = 1;

kp_theta = 1.3;
ki_theta = 0.04;
kd_theta = 1;

kd_yaw = 1;
ki_yaw = 1;
kp_yaw = 1;

kd = 1;
kp = 1;
ki = 1;

sample_freq = 250;

% Conversion for control signal to motorspeed(rpm)

Kv = 800;
Batt_Voltage = 11.1;
Zero_Point = 1100;