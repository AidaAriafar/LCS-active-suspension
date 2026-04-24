function y_out = simulate_closedloop_estimated_disturbance(G, H, K, Kd, LPF, r, yr, t)
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
H0 = dcgain(H);
LPFd = c2d(LPF, Ts, 'tustin');
[b_lpf, a_lpf] = tfdata(LPFd, 'v');
filt_states = zeros(max(length(a_lpf),length(b_lpf))-1,1);
x_real = zeros(4,1);
x_shadow = zeros(4,1);
y_out = zeros(N,1);
u = zeros(N,1);
xi = 0;
Kp = 200; Ki = 20000;
for i = 1:N
    y_real = C_out * x_real;
    y_out(i) = y_real;
    y_shadow = C_out * x_shadow;
    yr_hat = (y_real - y_shadow) / H0;
    [yr_hat_f, filt_states] = filter(b_lpf, a_lpf, yr_hat, filt_states);
    e = r(i) - y_real;
    xi = xi + Ts*e;
    u(i) = Kp*e + Ki*xi + Kd*yr_hat_f;
    x_real = Ad*x_real + Bd*u(i) + Wd*yr(i);
    x_shadow = Ad*x_shadow + Bd*u(i);
end
end