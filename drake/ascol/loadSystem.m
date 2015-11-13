function S = loadSystem(urdf, collision_groups, dt)
if nargin < 3
   dt = .001;
end

options.floating = true;
options.dt = dt;
options.terrain = RigidBodyFlatTerrain;
r = TimeSteppingRigidBodyManipulator(urdf, dt, options);
r = r.removeCollisionGroupsExcept(collision_groups);
r = compile(r);
v = r.constructVisualizer;
v.display_dt = 0.02;

S.nx = r.getNumStates();
S.nu = r.getNumInputs();
S.dt = dt;
S.r = r;
S.v = v;
end