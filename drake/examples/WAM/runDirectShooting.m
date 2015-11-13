function runDirectShooting()

%% Load the model 
S = loadSystem('urdf/wam7_bhand.urdf', {'base'}, .001);

%% Initial conditions and parameters
tf = 2;
nt = tf/S.dt;
S.ts = 0:S.dt:tf;
us = zeros(S.nu, nt);
S.x0 = zeros(S.nx,1);
S.x0(3) = -1;

% downsample residual?

% parametrize trajectory to reduce opti vars
S.nups = 10;
S.tps = 0:tf/S.nups:tf;
ups = zeros(S.nu, S.nups);

S.xf = S.x0;
S.Rs = diag([1 1 1 1 1    1 1 1 1 1    .1 .1 .1 .1 .1]);
S.Qs = eye(S.nx, S.nx);

% generate trajectory
%xs = sys_traj(S.x0, us, S);
c= arm_cost(ups, S)

%options = optimoptions('lsqnonlin', 'FinDiffRelStep',0.01, 'TolX',1e-2,'TolFun',1e-4, 'Display', 'iter');
%ups = lsqnonlin(@(us)arm_cost(ups, S), ups, [], [],options);

%% Play trajectory in visualizer
us = fromLinearInterp(ups, S.tps, S.ts(1:end-1));
xs = sys_traj(S.x0, us, S);
play_traj(xs, S.ts, S);
end


