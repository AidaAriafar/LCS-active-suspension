function u = controller_step(r,y,yr,Ts,reset)
persistent xi
if isempty(xi) || reset~=0
    xi = 0;
end
e = r - y;
xi = xi + Ts*e;
Kp = 200;
Ki = 20000;
Kd = -15000;
u = Kp*e + Ki*xi + Kd*yr;
end