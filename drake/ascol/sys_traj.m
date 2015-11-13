function [xs, As, Bs] = sys_traj(x0, us, S)
xs = x0;
t = 0;
if nargout >= 2
    As = zeros(S.nx, S.nx, size(us,2));
    Bs = zeros(S.nx, S.nu, size(us,2));
end

for i=1:length(us)
    
    if nargout < 2
        [xn] = S.r.update(t,xs(:,i),us(:,i));
    else
        [xn, df] = S.r.update(t,xs(:,i),us(:,i));
        As(:,:,i) = eye(nx) + df(:,1+(1:nx));
        Bs(:,:,i) = df(:,1+nx+(1:nu));
    end
    xs = [xs xn];
    t = t + S.dt;
end
end