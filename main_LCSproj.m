%main_LCS_project.m

clear; clc; close all

disp("SECTION 1: Parameters and State-Space Model")

M  = 300;
m  = 50;
k1 = 15000;
D  = 1000;
k2 = 150000;

A = [ 0  0  1  0
      0  0  0  1
     -k1/M  k1/M  -D/M   D/M
      k1/m -(k1+k2)/m  D/m  -D/m ];

B = [0;0; 1/M; -1/m];
W = [0;0; 0; k2/m];
C = [1 0 0 0];
Dyu = 0;
Dyw = 0;

sys_u  = ss(A,B,C,Dyu);
sys_wr = ss(A,W,C,Dyw);

disp("SECTION 2: Transfer Functions G(s), H(s)")

G = tf(sys_u);
H = tf(sys_wr);

G0 = dcgain(G);
H0 = dcgain(H);

disp("SECTION 3: Open-Loop Simulation (u=0) and PZMAP(G)")

t  = 0:0.001:10;
yr = 0.1 + 0.05*sin(10*t) + 0.02*cos(20*t);

y_open = lsim(sys_wr, yr, t);

figure; plot(t,y_open); grid on; xlabel("t"); ylabel("y"); title("Open-loop y(t), u=0")
figure; pzmap(G); grid on; title("Pole-Zero Map of G(s)")

disp("SECTION 4: PI Controller Design and Closed-Loop Tracking (yr=0 in design)")

Kp = 200;
Ki = 20000;
K_PI = tf([Kp Ki],[1 0]);

T_PI = feedback(G*K_PI, 1);

figure; step(T_PI); grid on; title("Closed-loop step (unit step) with PI")

disp("SECTION 5: Tracking with Reference and Road Disturbance, With/Without Kd Compensation")

R0 = 0.05;
r  = R0*ones(size(t));

Kd = -H0/G0;

u_noKd = zeros(size(t));
u_withKd = zeros(size(t));
y_noKd = zeros(size(t));
y_withKd = zeros(size(t));

for i=1:length(t)
    u_noKd(i)   = 0;
    u_withKd(i) = 0;
end

yr_vec = yr(:);
t_vec  = t(:);
r_vec  = r(:);

y_ref_PI = lsim(T_PI, r_vec, t_vec);

G_cl = feedback(G*K_PI,1);
S    = feedback(1, G*K_PI);

y_noKd   = lsim(G_cl, r_vec, t_vec) + lsim(S*H, yr_vec, t_vec);
y_withKd = lsim(G_cl, r_vec, t_vec) + lsim(S*(H + G*Kd), yr_vec, t_vec);

figure; plot(t_vec,y_noKd, t_vec,y_withKd); grid on
xlabel("t"); ylabel("y"); title("With/Without Kd (measured yr)")
legend("Kd=0","Kd=-H(0)/G(0)")

disp("SECTION 6: Lag Controller (Optional Alternative)")

z = 1.0025;
p = 0.6548;
Kc = 736.5045;
K_LAG = Kc*tf([1 z],[1 p]);

T_LAG = feedback(G*K_LAG,1);

figure; step(T_LAG); grid on; title("Closed-loop step (unit step) with Lag")
figure; pzmap(feedback(G*K_LAG,1)); grid on; title("PZMAP closed-loop with Lag")

y_noKd_lag   = lsim(feedback(G*K_LAG,1), r_vec, t_vec) + lsim(feedback(1,G*K_LAG)*H, yr_vec, t_vec);
y_withKd_lag = lsim(feedback(G*K_LAG,1), r_vec, t_vec) + lsim(feedback(1,G*K_LAG)*(H + G*Kd), yr_vec, t_vec);

figure; plot(t_vec,y_noKd_lag, t_vec,y_withKd_lag); grid on
xlabel("t"); ylabel("y"); title("Lag: With/Without Kd (measured yr)")
legend("Kd=0","Kd=-H(0)/G(0)")

disp("SECTION 7: Disturbance Estimation (yr not measurable) with Shadow Model + LPF")

tau = 0.05;
LPF = tf(1,[tau 1]);

y_model = lsim(sys_u, u_from_controller(K_PI, G, H, r_vec, zeros(size(yr_vec)), t_vec), t_vec);
y_meas  = y_withKd;

yr_hat = (y_meas - y_model)/H0;
yr_hat_f = lsim(LPF, yr_hat, t_vec);

y_est_comp = simulate_closedloop_estimated_disturbance(G,H,K_PI,Kd,LPF,r_vec,yr_vec,t_vec);

figure; plot(t_vec,yr_vec, t_vec,yr_hat, t_vec,yr_hat_f); grid on
xlabel("t"); ylabel("signal"); title("yr vs yr_hat vs filtered")
legend("yr","yr\_hat","yr\_hat\_f")

figure; plot(t_vec, y_noKd, t_vec, y_est_comp); grid on
xlabel("t"); ylabel("y"); title("Output: no-comp vs estimated-disturbance comp")
legend("Kd=0","Kd*yr_hat_f")

disp("SECTION 8: Robustness - Measurement Noise (-40 dB power)")

rng(1)
Pnoise_dB = -40;
Pnoise = 10^(Pnoise_dB/10);
n = sqrt(Pnoise)*randn(size(t_vec));

y_noise_comp = simulate_closedloop_meas_noise(G,H,K_PI,Kd,r_vec,yr_vec,t_vec,n);

figure; plot(t_vec, y_withKd, t_vec, y_noise_comp); grid on
xlabel("t"); ylabel("y"); title("Measured-noise effect")
legend("No noise","With noise")

disp("SECTION 9: Robustness - Param Uncertainty (+/-5%) for k1 and k2")

alphas = [-0.05 0 0.05];
Ys_k1 = zeros(length(t_vec), length(alphas));
Ys_k2 = zeros(length(t_vec), length(alphas));

for j=1:length(alphas)
    a = alphas(j);

    k1j = k1*(1+a);
    Aj = [ 0  0  1  0
           0  0  0  1
          -k1j/M  k1j/M  -D/M   D/M
           k1j/m -(k1j+k2)/m  D/m  -D/m ];
    sys_u_j  = ss(Aj,B,C,Dyu);
    sys_wr_j = ss(Aj,W,C,Dyw);
    Gj = tf(sys_u_j);
    Hj = tf(sys_wr_j);

    G0j = dcgain(Gj);
    H0j = dcgain(Hj);
    Kdj = -H0j/G0j;

    Ys_k1(:,j) = lsim(feedback(Gj*K_PI,1), r_vec, t_vec) + lsim(feedback(1,Gj*K_PI)*(Hj + Gj*Kdj), yr_vec, t_vec);
end

for j=1:length(alphas)
    a = alphas(j);

    k2j = k2*(1+a);
    Aj = [ 0  0  1  0
           0  0  0  1
          -k1/M  k1/M  -D/M   D/M
           k1/m -(k1+k2j)/m  D/m  -D/m ];
    Wj = [0;0;0;k2j/m];
    sys_u_j  = ss(Aj,B,C,Dyu);
    sys_wr_j = ss(Aj,Wj,C,Dyw);
    Gj = tf(sys_u_j);
    Hj = tf(sys_wr_j);

    G0j = dcgain(Gj);
    H0j = dcgain(Hj);
    Kdj = -H0j/G0j;

    Ys_k2(:,j) = lsim(feedback(Gj*K_PI,1), r_vec, t_vec) + lsim(feedback(1,Gj*K_PI)*(Hj + Gj*Kdj), yr_vec, t_vec);
end

figure; plot(t_vec, Ys_k1); grid on
xlabel("t"); ylabel("y"); title("Uncertainty in k1 (+/-5%)")
legend("k1-5%","k1 nominal","k1+5%")

figure; plot(t_vec, Ys_k2); grid on
xlabel("t"); ylabel("y"); title("Uncertainty in k2 (+/-5%)")
legend("k2-5%","k2 nominal","k2+5%")

disp("SECTION 10: Robustness - Combined Noise + Random (k1,k2) within +/-5%")

rng(2)
Nmc = 5;
Ymc = zeros(length(t_vec), Nmc);

for j=1:Nmc
    a1 = -0.05 + 0.10*rand;
    a2 = -0.05 + 0.10*rand;

    k1j = k1*(1+a1);
    k2j = k2*(1+a2);

    Aj = [ 0  0  1  0
           0  0  0  1
          -k1j/M  k1j/M  -D/M   D/M
           k1j/m -(k1j+k2j)/m  D/m  -D/m ];
    Wj = [0;0;0;k2j/m];

    sys_u_j  = ss(Aj,B,C,Dyu);
    sys_wr_j = ss(Aj,Wj,C,Dyw);
    Gj = tf(sys_u_j);
    Hj = tf(sys_wr_j);

    G0j = dcgain(Gj);
    H0j = dcgain(Hj);
    Kdj = -H0j/G0j;

    n = sqrt(Pnoise)*randn(size(t_vec));

    Ymc(:,j) = simulate_closedloop_meas_noise(Gj,Hj,K_PI,Kdj,r_vec,yr_vec,t_vec,n);
end

figure; plot(t_vec, Ymc); grid on
xlabel("t"); ylabel("y"); title("Combined: noise + param uncertainty (MC)")