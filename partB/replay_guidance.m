% REPLAY_GUIDANCE runs the guidance code offline on the logged estimate and
% obstacle memory, printing each detour decision and comparing the shift
% with the one recorded in the simulation

est  = out.est_log;                 % N x 3 [X Y yaw] at dt
obsl = out.obs_log;                 % 12 x 2 x N at dt
N = min(size(est,1), size(obsl,3));

clear Guidance_Control_replay       % reset persistent variables

sh = zeros(N,1);
for k = 1:N
    [~, ~, ~, ~, sh(k)] = Guidance_Control_replay(est(k,1), est(k,2), est(k,3), ...
        target_path, obsl(:,:,k), wall_grid);
end

figure; hold on; grid on
plot((0:N-1)*dt, sh, 'b-')
plot((0:numel(out.shift_log)-1)*dt, out.shift_log, 'r--')
xlabel('Time [s]'); ylabel('Avoidance shift [m]')
legend('Replay', 'Simulation')