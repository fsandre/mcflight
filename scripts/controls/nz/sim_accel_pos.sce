/*
 * Checking accelerometer postion and NMP zero in load factor response
 */
clear
exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');
exec('eqm/eqm_body.sci');
exec('eqm/stability_deriv.sci');
exec('eqm/stability_deriv_body.sci');

/* Setting parameters */
disp('Setting parameters to trim and initial state...')
V_ftps = 200;
alt_ft = 0;
xcg = 0.35;
[X0, controls, params] = trim_straight_level(V_ftps, alt_ft, xcg);

/* Initializing state in body axis */   
X0_body = X0; // this is in stability-axis, i.e, states:
              // X0(1)=V, X0(2)=alpha, X0(3)=beta
X0_body(1) = params.VT_ftps*cos(X0(2))*cos(X0(3));
X0_body(2) = params.VT_ftps*sin(X0(3));
X0_body(3) = params.VT_ftps*sin(X0(2))*cos(X0(3));

controls_trim = controls;

/* Calculating stability derivatives for conditon */
disp('Calculating stability derivatives for conditon...');
[long_deriv_body, lat_deriv_body] = stability_deriv_body(eqm_body, X0_body, controls_trim, params);

/* Linearizing... */
disp('Linearizing...');
X0_lin_body = X0([
                  1    //u_ftps
                  2    //w_ftps
                  5    //theta_rad
                  8    //q_rps
                 ]);
                 
U0 = [ controls_trim.throttle
       controls_trim.elev_deg];
                 
function [y,xd] = sim_f16_body(X,U)
    controls.throttle = U(1);
    controls.elev_deg = U(2);
    controls.ail_deg = 0.0;
    controls.rudder_deg = 0.0;
    X_full = zeros(13,0);
    X_full(1) = X(1);
    X_full(3) = X(2);
    X_full(5) = X(3);
    X_full(8) = X(4);
    X_full(13) = tgear(U(1));
    [xd_full,outputs] = eqm_body(0, X_full, controls, params);
    xd(1) = xd_full(1);
    xd(2) = xd_full(3);
    xd(3) = xd_full(5);
    xd(4) = xd_full(8);
    outputs.nzs_g = outputs.nx_g*sin(outputs.alpha_deg/180*%pi) + outputs.nz_g*cos(outputs.alpha_deg/180*%pi);
    y = [
        outputs.nx_g;
        outputs.ny_g;
        outputs.nz_g; 
        outputs.alpha_deg;
        outputs.q_rps;
        outputs.Q_lbfpft2;
        outputs.mach;
        outputs.nzs_g; // The sign of Nz is being considered positive up here
        ];
endfunction
[A_body,B_body,C_body,D_body] = lin(sim_f16_body, X0_lin_body, U0);

l_arm_ft = long_deriv_body.Zelev/long_deriv_body.Melev;
disp('Including nz (stability) on different position as state-space output: '+string(l_arm_ft)+' ft.');
// The sign of Nz is being considered positive up here
C_body(9,:) = C_body(8,:) + l_arm_ft/params.g0_ftps2*A_body(4,:);
D_body(9,:) = D_body(8,:) + l_arm_ft/params.g0_ftps2*B_body(4,:);
ss_body = syslin("c", A_body, B_body, C_body, D_body);

disp('Simulating linear model...');
t = 0:0.001:3;
function u = elev_step_lin(t)
    if(t<0.5) then
        u = 0;
    elseif (t>=0.5 && t<=0.53)
        u = - 1/0.03*(t-0.5);
    else
        u = -1;
    end
endfunction
[y_body,x_body] = csim(elev_step_lin, t, ss_body(:,2));
