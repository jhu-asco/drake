function runDynamicsWithJacs
% Simulate the (passive) dynamics of the Atlas model 

%% Load the model with a floating base
dt = .001;
options.floating = true;
options.dt = dt;
options.terrain = RigidBodyFlatTerrain;
r = Atlas('urdf/atlas_convex_hull.urdf', options);
%r = TimeSteppingRigidBodyManipulator('urdf/atlas_convex_hull.urdf', dt, options);
%r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
r = compile(r);
nx = r.getNumStates();
nu = r.getNumInputs();

%% Initialize the viewer
v = r.constructVisualizer;
v.display_dt = 0.02;

%% Step through dynamics
x0 = Point(r.getStateFrame());
x0 = resolveConstraints(r,x0);

u = zeros(nu,1);
x = x0.double();
xs = [x];
ts = [0];
As = [];
Bs = [];
for t = dt:dt:.6; 
    t
    [xn] = r.update(t,x,u);  
    %[xn,df] = r.update(t,x,u);  
    x = xn;
    %A = eye(nx) + df(:,1+(1:nx));
    %B = df(:,1+nx+(1:nu));
    
    xs = [xs x];
    ts = [ts t];
    %As = [As A];
    %Bs = [Bs B];
end


%% Display traj
trj = DTTrajectory(ts, xs);
trj = setOutputFrame(trj,r.getStateFrame());
v.playback(trj);
end
