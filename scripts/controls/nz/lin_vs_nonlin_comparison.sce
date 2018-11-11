/*
 * Checking accelerometer postion and NMP zero in load factor response
 */
clear
exec('eqm/eqm_body.sci');
exec('eqm/params_f16.sci');
exec('eqm/stability_deriv.sci');
exec('eqm/stability_deriv_body.sci');

params = load_f16();
exec('controls/nz/trim_v502_alt0_xcg35_level.sce');

disp('Building state-space...');
controls_trim = controls;
X0_lin_body = X0_body([
                  1    //u_ftps
                  3    //w_ftps
                  5    //theta_rad
                  8    //q_rps
                 ]);
                 
U0 = [ controls_trim.throttle
       controls_trim.elev_deg];

rad2deg = 180/%pi;

/* Initializing state in body axis */   
X0_body = X0;
X0_body(1) = params.VT_ftps*cos(X0(2))*cos(X0(3));
X0_body(2) = params.VT_ftps*sin(X0(3));
X0_body(3) = params.VT_ftps*sin(X0(2))*cos(X0(3));

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
function xd = f16_model_body(t,X)
    [xd] = eqm_body(t, X, controls, params);
endfunction
t = 0:0.001:3;
controls.elev_deg = elev_step;
y_body = ode(X0_body, t(1), t, f16_model_body);

ss_body = syslin("c", A_body, B_body, C_body, D_body);

disp('Simulating linear model...');
t = 0:0.001:3;
function u = elev_step_lin(t)
    u = elev_step(t) - controls_trim.elev_deg;
endfunction
[y_lin_body,x_lin_body] = csim(elev_step_lin, t, ss_body(:,2));

disp('Comparing body and stability axis non-linear simulations');
f2 = scf(2);xgrid;xlabel('time(s)');ylabel('elevator (deg)');
title('Elevator input')
plot(t, elev_step);
f3 = scf(3);xgrid;xlabel('time(s)');ylabel('theta(deg)');
title('Theta linear x non-linear')
plot(t, y_body(5,:)*rad2deg, t, (x_lin_body(3,:) + X0_body(5))*rad2deg); // theta plot
legend('Non-linear','Linear');
f4 = scf(4);xgrid;xlabel('time(s)');ylabel('pitch rate(deg/s)');
title('Pitch rate linear x non-linear')
plot(t, y_body(8,:)*rad2deg, t, (x_lin_body(4,:) + X0_body(8))*rad2deg); // pitch rate plot
legend('Non-linear','Linear');
