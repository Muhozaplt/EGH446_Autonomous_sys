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
set_Q= 0.07;
Q = set_Q * eye(6);

% Define the measurement noise covariance matrix
set_R = 100;
R = set_R * eye(3);
% Initial state and covariance
x0 = zeros(6,1);
P0 = eye(6);

