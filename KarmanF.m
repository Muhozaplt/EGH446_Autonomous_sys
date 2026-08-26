dt=.01;
A= [1 0 0 dt 0 0 ;
    0 1 0 0 dt 0 ;
    0 0 1 0 0 dt ;
    0 0 0 1 0 0 ; 
    0 0 0 0 1 0 ;
    0 0 0 0 0 1 ];

H= [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0];

% Define the process noise covariance matrix
set_Q= 0.5;
Q = set_Q * eye(6);

% Define the measurement noise covariance matrix
set_R = 10;
R = set_R * eye(3);
% Initialize state vector and covariance matrix
x = zeros(6, 1); % Initial state
P = eye(6);      % Initial covariance matrix
