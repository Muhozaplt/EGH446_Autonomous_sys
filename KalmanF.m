dt = 0.1;
% the states are defined as follows: x, y, yaw, x_dot, y_dot, yaw_dot
A = [1 0 0 dt 0 0 ;
     0 1 0 0 dt 0 ;
     0 0 1 0 0 dt ;
     0 0 0 1 0 0 ;
     0 0 0 0 1 0 ;
     0 0 0 0 0 1 ];

H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

% Define the process noise covariance matrix
Q = diag([0.001 0.001 0.001 0.05 0.05 0.05]);


% Measurement noise covariance — with the given sensor spec
sigma_x2     = 0.1;                 % m^2
sigma_y2     = 0.1;                 % m^2
sigma_theta2 = 2* (pi/180)^2;      % deg^2 -> rad^2
R = diag([sigma_x2, sigma_y2, sigma_theta2]);

% Initial state and covariance
P0 =eye(6);
