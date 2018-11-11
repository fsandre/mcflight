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
xcg = 0.35;
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
        outputs.alpha_deg;
        outputs.q_rps*rad2deg;
        ];
endfunction
[A_stab,B_stab,C_stab,D_stab] = lin(sim_f16, X0_lin, U0);

// Including actuator as a simple-lag filter 
// with time constant tau_act = 1/20.2
// and an alpha filter tau_alpha = 0.1 s
tau_act = 1/20.2;
tau_alpha = 0.1;
A_aug = [A_stab            -B_stab(:,2)    zeros(4,1)
         0 0           0 0 -1/tau_act      0
         0 1/tau_alpha 0 0  0              -1/tau_alpha];
B_aug = [B_stab(:,1) zeros(4,1)
         0           1/tau_act
         0           0];
C_aug = [C_stab      zeros(2,2)
         zeros(1,4)  0   rad2deg];
D_aug = [D_stab
         0   0];
ss_aug = syslin("c", A_aug, B_aug, C_aug, D_aug);
//evans(ss_aug(3,2),10);

//closing loop at alpha filter
k_alpha = 0.5;
A_cl_alpha = A_aug - B_aug(:,2)*k_alpha*C_aug(3,:);
ss_cl_alpha = syslin("c", A_cl_alpha, B_aug, C_aug, D_aug);
evans(ss_cl_alpha(2,2),10);

k_q = 0.2;
