function u = u_from_controller(K, G, ~, r, ~, t)
T_u = feedback(K, G);
u = lsim(T_u, r, t);
end