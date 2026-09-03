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
set_Q= 0.001;
Q = set_Q * eye(6);


% Measurement noise covariance — set directly from the given sensor spec
sigma_x2     = 0.1;                 % m^2
sigma_y2     = 0.1;                 % m^2
sigma_theta2 = 2* (pi/180)^2;      % deg^2 -> rad^2
R =diag([1000+sigma_x2, 500+sigma_y2, sigma_theta2]);

% Initial state and covariance
x0 = zeros(6,1);
P0 =eye(6);
