/*
 * Pitch axis stability augmentation example
 */
clear
exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');
exec('eqm/stability_deriv.sci');
exec('eqm/stability_deriv_body.sci');

/* Setting parameters */
disp('Setting parameters to trim and initial state...')
V_ftps = 502;
alt_ft = 0;
xcg = 0.3;
g0_fps2 = atmos.g0_mps2/0.3048;
[X0, controls, params] = trim_straight_level(V_ftps, alt_ft, xcg);
rad2deg = 180/%pi;

controls_trim = controls;

function y = elev_step(t)
    if(t<0.5) then
        y = controls_trim.elev_deg;
    elseif (t>=0.5 && t<=0.53)
        y = controls_trim.elev_deg - 1/0.03*(t-0.5);
    else
        y = controls_trim.elev_deg-1;
    end
endfunction
function xd = f16_model(t,X)
    [xd] = eqm(t, X, controls, params);
endfunction
t = 0:0.001:3;
controls.elev_deg = elev_step;
y = ode(X0, t(1), t, f16_model);

disp('Linearizing...');
X0_lin = X0([
                  1    //V_ftps
                  2    //alpha_rad
                  5    //theta_rad
                  8    //q_rps
                 ]);
                 
U0 = [ controls_trim.throttle
       controls_trim.elev_deg];
                 
function [y,xd] = sim_f16(X,U)
    controls.throttle = U(1);
    controls.elev_deg = U(2);
    controls.ail_deg = 0.0;
    controls.rudder_deg = 0.0;
    X_full = zeros(13,0);
    X_full(1) = X(1);
    X_full(2) = X(2);
    X_full(5) = X(3);
    X_full(8) = X(4);
    X_full(13) = tgear(U(1));
    [xd_full,outputs] = eqm(0, X_full, controls, params);
    xd(1) = xd_full(1);
    xd(2) = xd_full(2);
    xd(3) = xd_full(5);
    xd(4) = xd_full(8);

    y = [
        X(1)
        X(2)
        X(3)
        X(4)
        outputs.nx_g;
        outputs.ny_g;
        outputs.nz_g;
        ];
endfunction
[A_stab,B_stab,C_stab,D_stab] = lin(sim_f16, X0_lin, U0);
[Y0, XD0] = sim_f16(X0, U0);
// Including actuator as a simple-lag filter 
// with time constant tau_act = 1/20.2
// and an alpha filter tau_alpha = 0.1 s
tau_act = 1/20.2;
tau_alpha = 0.1;
alpha_trim_rad = X0_lin(2);
theta_trim_rad = X0_lin(3);
q_trim_rps = X0_lin(4);
V_trim_fps = V_ftps;
omega_tas_cf = 1;
zeta_tas_cf = 1;
omega_alpha_cf = 5;
zeta_alpha_cf = 1;
nxb_trim_g = Y0(5);
nyb_trim_g = Y0(6);
nzb_trim_g = Y0(7);
nzs_trim_g = nzb_trim_g*cos(alpha_trim_rad) + nxb_trim_g*sin(alpha_trim_rad);
nxs_trim_g = nxb_trim_g*cos(alpha_trim_rad) - nzb_trim_g*sin(alpha_trim_rad); 

// Gains
Kint = 0.5;
KP = 0.08;
Kq = 0.5;
KD = 0;
Ktheta = 0;
Ktasdot = 0.005;
KV = 0.002;
//Ktasdot = 0;
//Ktheta = 0.5;
//KV = 0.002;
