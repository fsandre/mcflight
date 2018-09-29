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
y = ode(X0, t(1), t, f16_model);
y_body = ode(X0_body, t(1), t, f16_model_body);

f1 = scf(1);
plot(t, y(5,:), t, y_body(5,:));
f2 = scf(2);
plot(t, y(8,:), t, y_body(8,:));
