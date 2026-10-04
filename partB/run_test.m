% RUN_TEST runs one simulation and evaluates it

setupPartB
KalmanF

replan_history = {};

out = sim('sl_quadrotorDynamics', 'StopTime', '600', 'ReturnWorkspaceOutputs', 'on');
r = eval_run(out, target_path, logical_map, obstacles)