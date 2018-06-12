exec('models/mass_geom_data.sci');
exec('models/atmos_constants.sci');
params = load_aircraft_params();
geom = params.geom;
mass = params.mass;

g0_ftps2 = 32.17;
rad2deg = 57.29578;
ft2m = 0.3048;
kn2mps = 0.514444;
pa2lbfpft2 = 0.0208854;

initial.VT_ftps = 0.0;
initial.alpha_rad = 0.0;
initial.beta_rad = 0.0;
initial.phi_rad = 0.0;
initial.theta_rad = 0.0;
initial.psi_rad = 0.0;
initial.p_rps = 0.0;
initial.q_rps = 0.0;
initial.r_rps = 0.0;
initial.north_ft = 0.0;
initial.east_ft = 0.0;
initial.alt_ft = 0.0;

