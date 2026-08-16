%% close previously open model
close_system('sl_quadrotorDynamics',0);
 


%% add toolboxes to path
homedir = "/home/muhoza/EGH446_Autonomous_sys"; 
addpath(genpath(strcat(homedir,[filesep,'toolboxes'])));
cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);

%% open current model
open_system('sl_quadrotorDynamics'); % quadrotor aerial  model


cd(homedir);




