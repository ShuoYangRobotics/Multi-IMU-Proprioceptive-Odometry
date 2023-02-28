function xn1 = dyn_euler(xn, un, un1, dt, dynfunc)

if ~iscolumn(xn)
    xn = xn';
end
if ~iscolumn(un)
    un = un';
end
if ~iscolumn(un1)
    un1 = un1';
end

xn1 = dynfunc(xn,un)*dt + xn;

end