%% close previously open model
close_system('sl_quadrotorDynamics',0);
 


%% add toolboxes to path
homedir = pwd; 
addpath(genpath(strcat(homedir,[filesep,'toolboxes'])));
cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

%% adding PartB path
addpath(fullfile(homedir, 'partB'));

cd(homedir);

%% open current model
open_system('sl_quadrotorDynamics'); % quadrotor aerial  model


cd(homedir);

scanAngles = linspace(-pi / 4, pi / 4, 10);


