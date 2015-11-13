function y = arm_cost(ups, S)
% This evaluates the arm costs in least-squares form,
% i.e. the residuals at each time-step
disp('cost')
us = fromLinearInterp(ups, S.tps, S.ts);
xs = sys_traj(S.x0, us, S);%Generate the system trajectory at given controls

N=size(us,2);

y=[];%y is the residual of the cost function
for k=1:N
  y = [y; S.Rs*us(:,k)];
end

for k=2:N
  y = [y; S.Qs*xs(:,k)];
end
end