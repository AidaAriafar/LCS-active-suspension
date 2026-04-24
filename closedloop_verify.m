clc; clear; close all;

cfg = coder.config('mex');
cfg.TargetLang = 'C';
cfg.EnableMemcpy = true;
codegen -config cfg controller_step_entry -args {0,0,0,0,0}

M = 300;
m = 50;
k1 = 15000;
D = 1000;
k2 = 150000;

A = [0 0 1 0;
     0 0 0 1;
    -k1/M  k1/M  -D/M   D/M;
     k1/m -(k1+k2)/m  D/m -D/m];

B_u  = [0; 0;  1/M; -1/m];
B_yr = [0; 0;  0;   k2/m];

C = [1 0 0 0];

Ts = 0.001;
t = 0:Ts:5;
N = numel(t);

Ad = eye(4) + A*Ts;
Bd_u = B_u*Ts;
Bd_yr = B_yr*Ts;

R0 = 0.05;
r = R0*ones(1,N);
yr = 0.1 + 0.05*sin(10*t) + 0.02*cos(20*t);

xM = zeros(4,1);
xX = zeros(4,1);

yM = zeros(1,N);
yX = zeros(1,N);

uM = zeros(1,N);
uX = zeros(1,N);

for k=1:N
    reset = double(k==1);

    yM(k) = C*xM;
    yX(k) = C*xX;

    uM(k) = controller_step(r(k),yM(k),yr(k),Ts,reset);
    uX(k) = controller_step_entry_mex(r(k),yX(k),yr(k),Ts,reset);

    xM = Ad*xM + Bd_u*uM(k) + Bd_yr*yr(k);
    xX = Ad*xX + Bd_u*uX(k) + Bd_yr*yr(k);
end

eps_max = max(abs(uM - uX));
eps_max