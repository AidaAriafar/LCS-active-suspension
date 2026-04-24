function y_out = simulate_closedloop_meas_noise(G, H, K, Kd, r, yr, t, n)
Ts = t(2)-t(1);
N = length(t);
M = 300; m = 50; k1 = 15000; D = 1000; k2 = 150000;
A = [0 0 1 0; 0 0 0 1; -k1/M k1/M -D/M D/M; k1/m -(k1+k2)/m D/m -D/m];
B = [0;0;1/M;-1/m];
W = [0;0;0;k2/m];
C_out = [1 0 0 0];
Ad = eye(4) + A*Ts;
Bd = B*Ts;
Wd = W*Ts;
x = zeros(4,1);
y_out = zeros(N,1);
u = zeros(N,1);
xi = 0;
Kp = 200; Ki = 20000;
for i = 1:N
    y_real = C_out * x;
    y_meas = y_real + n(i);
    e = r(i) - y_meas;
    xi = xi + Ts*e;
    u(i) = Kp*e + Ki*xi + Kd*yr(i);
    x = Ad*x + Bd*u(i) + Wd*yr(i);
    y_out(i) = y_real;
end
end