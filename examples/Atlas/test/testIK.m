function testIK

addpath('..');
options.floating = true;
options.dt = 0.001;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);
v = r.constructVisualizer();

cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
options = struct();
cost = double(cost);
options.Q = diag(cost(1:r.getNumDOF));

mexoptions = options;
mexoptions.use_mex = true;
options.use_mex = false;

% set initial state to fixed point
load('../data/atlas_fp.mat');

q0 = xstar(1:r.getNumDOF);

q = inverseKin(r,q0,options);
qmex = inverseKin(r,q0,mexoptions);
valuecheck(qmex,q,1e-5);
v.draw(0,[q;0*q]); drawnow;

q = inverseKin(r,q0,0,[0;0;2],options);
qmex = inverseKin(r,q0,0,[0;0;2],mexoptions);
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

r_foot = r.findLink('r_foot');
q = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;-.1;.2],options);
tic
qmex = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;-.1;.2],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

q = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;-.1;.2;0;0;0],options);
tic
qmex = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;-.1;.2;0;0;0],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

q = inverseKin(r,q0,0,[0;0;nan],r_foot,[0;-.1;.2;0;0;0],options);
tic
qmex = inverseKin(r,q0,0,[0;0;nan],r_foot,[0;-.1;.2;0;0;0],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;
