exec('models/eqm_data.sci');
exec('models/engine_data.sci');
exec('models/aerodynamics_data.sci');

exec('trim/trim_f16.sci');
v_ref = 220;
[X, controls, params] = trim_straight_level(v_ref);
initial.VT_ftps = X(1);
initial.alpha_rad = X(2);
initial.beta_rad = X(3);
initial.phi_rad = X(4);
initial.theta_rad = X(5);
initial.psi_rad = X(6);
initial.p_rps = X(7);
initial.q_rps = X(8);
initial.r_rps = X(9);
initial.north_ft = X(10);
initial.east_ft = X(11);
initial.alt_ft = X(12);
initial.throttle_pow = X(13);



