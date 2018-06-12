//<-- NO CHECK REF -->
exec('models/aircraft_data.sci');

X0 = [
   200.
   0.3438075
   0.
   0.
   0.3438075
   0.
   0.
   0.
   0.
   0.
   0.
   0.
   18.607002
   ];
initial.VT_ftps = X0(1);
initial.alpha_rad = X0(2);
initial.beta_rad = X0(3);
initial.phi_rad = X0(4);
initial.theta_rad = X0(5);
initial.psi_rad = X0(6);
initial.p_rps = X0(7);
initial.q_rps = X0(8);
initial.r_rps = X0(9);
initial.north_ft = X0(10);
initial.east_ft = X0(11);
initial.alt_ft = X0(12);
initial.throttle_pow = X0(13);

controls.throttle = 0.286526;
controls.elev_deg = 0.7233067;
controls.ail_deg = 0.0;
controls.rudder_deg = 0.0;

xcos('tests/models/aircraft_test.zcos');
