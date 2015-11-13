function runDynamicsWithJacs()
% Simulate the (passive) dynamics of the WAM model 

%% Load the model with a floating base
S = loadSystem('urdf/wam7_bhand.urdf', {'base'});

%% Step through dynamics
x0p = Point(S.r.getStateFrame());

jidx = [7:21];
u = zeros(S.nu,1);
%xjd = zeros(15,1);
xjd = zeros(length(jidx),1);
xjd(2) = pi/10;
xjd(3) = pi/8;
xjd(4) = pi/8;
%xjd(6) = pi/2;
% base_yaw shoulder_pitch shoulder_yaw elbow_pitch wrist_yaw wrist_pitch
kp = diag([50 50 50  60   5.0   5.0    1.0  0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01]);%0.01 0.01  1 0 0 ]);
kd = diag([12 30 10  10    .2   0.3     0     0  0 0 0 0 0 0 0 ]);%0    0  0 0 0 ]);
                                          
x = x0p.double();
x(3) = -1;

xs = x;
ts = 0;
As = [];
Bs = [];

for t = S.dt:S.dt:5
    
    [xn] = S.r.update(t,x,u);  
    x = xn;

    x(1:6) = zeros(6,1); x(3) = -1; % Keep base fixed
    
    xj = x(jidx);
    xjv = x(jidx + 21);
    u(jidx-6) = kp*(xjd-xj) + kd*(-xjv);
    
    %A = eye(nx) + df(:,1+(1:nx));
    %B = df(:,1+nx+(1:nu));
    
    xs = [xs x];
    ts = [ts t];
    %As = [As A];
    %Bs = [Bs B];
end

%% Display traj
play_traj(xs, ts, S);
end


