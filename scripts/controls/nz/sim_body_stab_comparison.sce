/*
 * Checking accelerometer postion and NMP zero in load factor response
 */
clear
exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');
exec('eqm/eqm_body.sci');

/* Setting parameters */
disp('Setting parameters to trim...')
params = load_f16();
params.xcg = .35;
params.coordinated_turn = 0;
params.turn_rate_rps = 0.0;
params.roll_rate_rps = 0.0;
params.pitch_rate_rps = 0.0;
params.phi_rad = 0.0;
params.gamma_rad = 0.0;
params.stability_axis_roll = 0;
params.VT_ftps = 502;
params.alt_ft = 0;
function y = costf16(x)
    y = cost_trim_f16(x,params);
endfunction
S0 = [
     .0   //throttle 0-1
     0.0  //elev_deg
     0.0  //alpha_rad
     //0.0//ail_deg
     //0.0//rudder_deg
     //0.0//beta_rad
     ];
S = fminsearch(costf16, S0);
function y = elev_step(t)
    if(t<0.5) then
        y = S(2);
    elseif (t>=0.5 && t<=0.53)
        y = S(2) - 1/0.03*(t-0.5);
    else
        y = S(2)-1;
    end
endfunction
disp('Trimmed successfully: ');
disp(S);

/* Initializing state to simulate */
disp('Initializing state to simulate...')
X0 = [
      params.VT_ftps    //(1)VT_fps
      S(3)              //(2)alpha_rad
      0.0               //(3)beta_rad
      0.0               //(4)phi_rad
      S(3)              //(5)theta_rad
      0.0               //(6)psi_rad
      0.0               //(7)p_rps
      0.0               //(8)q_rps
      0.0               //(9)r_rps
      0.0               //(10)north position ft
      0.0               //(11)east position ft
      params.alt_ft     //(12)alt_ft
      tgear(S(1))       //(13)power_perc
     ];

controls.throttle = S(1);
controls.elev_deg = S(2);
controls.ail_deg = 0.0;
controls.rudder_deg = 0.0;
function xd = f16_model(t,X)
    [xd] = eqm(t, X, controls, params);
endfunction
t = 0:0.001:3;
controls.elev_deg = elev_step;
y = ode(X0, t(1), t, f16_model);

/* Simulating in body-axis */
/*disp('Simulating in body-axis...');
X0_body = [
      params.VT_ftps*cos(S(3))    //(1)u_ftps
      0.0                         //(2)v_ftps
      params.VT_ftps*sin(S(3))    //(3)w_ftps
      0.0                         //(4)phi_rad
      S(3)                        //(5)theta_rad
      0.0                         //(6)psi_rad
      0.0                         //(7)p_rps
      0.0                         //(8)q_rps
      0.0                         //(9)r_rps
      0.0                         //(10)north position ft
      0.0                         //(11)east position ft
      params.alt_ft               //(12)alt_ft
      tgear(S(1))                 //(13)power_perc
     ];

function xd = f16_model_body(t,X)
    [xd] = eqm_body(t, X, controls, params);
endfunction
y_body = ode(X0_body, t(1), t, f16_model_body);*/

/* Setting different possible accelerometer positions */
disp('Setting different possible accelerometer positions...')
x_acc_ft = [0 5 6 6.1 7 15];
len_acc = length(x_acc_ft);
az_acc_g = zeros(length(t), length(x_acc_ft));
nz_g = 0*t;
nx_g = 0*t;
nzs_g = 0*t;
mach = 0*t;
thrust_pound = 0*t;
/* Calculating further output parameters */
/*for i=1:length(t)
    [xd,outputs] = eqm(t(i), y(:,i), controls, params);
    nz_g(i) = outputs.nz_g;
    nx_g(i) = outputs.nx_g;
    nzs_g(i) = nx_g(i)*sin(y(2,i))+nz_g(i)*cos(y(2,i));
    mach(i) = outputs.mach;
    thrust_pound(i) = outputs.thrust_pound*sin(y(5,i));
    for j=1:len_acc
        az_acc_g(i,j) = nzs_g(i) + xd(8)*x_acc_ft(j)/params.g0_ftps2;
    end
end*/

/* Linearizing... */
disp('Linearizing...');
l_arm_ft = 15;
X0_Valpha = [
      params.VT_ftps    //VT_fps
      S(3)              //alpha_rad
      S(3)              //theta_rad
      0.0               //q_rps
     ];
U0 = [
     S(1) //throttle
     S(2) //elevator
     ];
function [y,xd] = sim_f16_Valpha(X,U)
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
    xd = xd_full([1 2 5 8]);
    outputs.nzs_g = outputs.nx_g*sin(X(2)) + outputs.nz_g*cos(X(2));
    y = [
        outputs.nz_g;
        outputs.ny_g;
        outputs.nx_g;
        outputs.Q_lbfpft2;
        outputs.mach;
        outputs.q_rps;
        outputs.alpha_deg;
        outputs.nzs_g;
        outputs.nzs_g + xd(4)*l_arm_ft/params.g0_ftps2;
        ];
endfunction
X0_uw = [
      params.VT_ftps*cos(S(3))    //u_ftps
      params.VT_ftps*sin(S(3))    //w_ftps
      S(3)                        //theta_rad
      0.0                         //q_rps
     ];
function [y,xd] = sim_f16_uw(X,U)
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
        outputs.nz_g;
        outputs.ny_g;
        outputs.nx_g;
        outputs.Q_lbfpft2;
        outputs.mach;
        outputs.q_rps;
        outputs.alpha_deg;
        outputs.nzs_g;
        outputs.nzs_g + xd(4)*l_arm_ft/params.g0_ftps2;
        ];
endfunction
[A,B,C,D] = lin(sim_f16_Valpha, X0_Valpha, U0);
ss_Valpha = syslin("c", A, B, C, D);
[A,B,C,D] = lin(sim_f16_uw, X0_uw, U0);
ss_uw = syslin("c", A, B, C, D);

disp('Simulating linear model...');
function u = elev_step_lin(t)
    u = elev_step(t)-S(2)
endfunction
[y_Valpha,x_Valpha] = csim(elev_step_lin,t,ss_Valpha(7,2));
[y_uw,x_uw] = csim(elev_step_lin,t,ss_uw(7,2));
