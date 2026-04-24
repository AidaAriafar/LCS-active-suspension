clc; clear; close all;

cfg = coder.config('mex');
codegen -config cfg controller_step_entry -args {0,0,0,0,0}

Ts = 0.001;
t = 0:Ts:5;
N = numel(t);

R0 = 0.05;
r = R0*ones(1,N);
yr = 0.1 + 0.05*sin(10*t) + 0.02*cos(20*t);

u_mat = zeros(1,N);
u_mex = zeros(1,N);

y_sig = zeros(1,N);
y_sig(1) = 0;

for k=1:N
    reset = double(k==1);
    u_mat(k) = controller_step(r(k),y_sig(k),yr(k),Ts,reset);
    u_mex(k) = controller_step_entry_mex(r(k),y_sig(k),yr(k),Ts,reset);
    if k < N
        y_sig(k+1) = y_sig(k);
    end
end

eps_max = max(abs(u_mat - u_mex));
eps_max