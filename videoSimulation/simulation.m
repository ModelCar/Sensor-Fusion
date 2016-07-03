%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename: generation.m
% Authors: Fang Yuan
% Description: generates simulated measurement data of ultrasonic sensor and
% depth map. The data is used for by videosimulation.cpp 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;

% number of samples
N = 500;

% sampling periode
T = 0.1;

% sensor distance
Distance = 0.15;

% system noise  
w1 = 0.002;
w2 = 0.01;
w3 = 0.025;

% measurement noise of ultrasonic sensor
v1 = 0.15;      
v2 = 0.15;
v3 = 0.15; 

% measurement noise of depth map
vd = 0.81;  

% time vector generation
time = zeros(1,N);
for t=1:N
    time(t) = t*T;
end

% generation of the control vector(not used)
F = zeros(1,N);

% initial conditions [p v a]
q0 = [34; -9.5; 1.5];

% simulation and generation of the real vehicle state
trajectory = zeros(3,N);
trajectory(:,1) = q0;
for t=2:N
    trajectory(1,t) = trajectory(1,t-1) + trajectory(2,t-1) * T + trajectory(3,t-1) * 0.5* T * T + random('Normal', 0, w1);
    trajectory(2,t) = trajectory(2,t-1) + trajectory(3,t-1) * T + random('Normal', 0, w2);
    trajectory(3,t) = trajectory(3,t-1) + random('Normal', 0, w3);
end

% generation of measures
measure_ultrasonic = zeros(21,N);
measure_depthmap = zeros(1,N);
r_obj = 0.01;
for t=1:N
    % 1st measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(1,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(2,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(3,t) = m3 + random('Normal', 0, v3);
    
    % 2nd measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(4,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(5,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(6,t) = m3 + random('Normal', 0, v3);
    
    % 3rd measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(7,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(8,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(9,t) = m3 + random('Normal', 0, v3);
    
    % 4th measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(10,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(11,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(12,t) = m3 + random('Normal', 0, v3);
    
    % 5th measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(13,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(14,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(15,t) = m3 + random('Normal', 0, v3);
    
    
    % 6th measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(16,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(17,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(18,t) = m3 + random('Normal', 0, v3);
    
    % 7th measurement set
    theta = rand(1) * pi/20;
    r = trajectory(1,t)/cos(theta);
    x = (r + r_obj) * sin(theta);
    y = (r + r_obj) * cos(theta);
    r2 = sqrt((x - Distance) * (x - Distance) + y * y) - r_obj;
    r3 = sqrt((x + Distance) * (x + Distance) + y * y) - r_obj;

    m1 = r*2;
    m2 = sqrt(r2*r2*4 + Distance*Distance);
    m3 = sqrt(r3*r3*4 + Distance*Distance);
    
    measure_ultrasonic(19,t) = m1 + random('Normal', 0, v1);
    measure_ultrasonic(20,t) = m2 + random('Normal', 0, v2);
    measure_ultrasonic(21,t) = m3 + random('Normal', 0, v3);
      
    % depth map
    measure_depthmap(1,t) = trajectory(1,t) + random('Normal', 0, vd);
end

% write to file
fid = fopen('sensor_data.m', 'w');

exportMatlab(fid, 'F', F);
exportMatlab(fid, 'measure_ultrasonic', measure_ultrasonic);
exportMatlab(fid, 'measure_depthmap', measure_depthmap);
exportMatlab(fid, 'trajectory', trajectory);

fclose(fid);