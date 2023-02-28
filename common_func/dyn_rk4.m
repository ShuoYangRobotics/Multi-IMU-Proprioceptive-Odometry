function xn1 = dyn_rk4(xn, un, un1, dt, dynfunc)

if ~iscolumn(xn)
    xn = xn';
end
if ~iscolumn(un)
    un = un';
end
if ~iscolumn(un1)
    un1 = un1';
end

k1 = dynfunc(xn,un);
k2 = dynfunc(xn+dt*k1/2,(un+un1)/2);
k3 = dynfunc(xn+dt*k2/2,(un+un1)/2);
k4 = dynfunc(xn+dt*k3,un1);

xn1 = xn + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);


end