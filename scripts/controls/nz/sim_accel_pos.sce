/*
 * Checking accelerometer postion and NMP zero in load factor response
 */
clear
exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');

/* Setting parameters to trim */
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

/* Initializing state to simulate */
disp('Initializing state to simulate...')
X0 = [
      params.VT_ftps    //VT_fps
      S(3)              //alpha_rad
      0.0               //beta_rad
      0.0               //phi_rad
      S(3)              //theta_rad
      0.0               //psi_rad
      0.0               //p_rps
      0.0               //q_rps
      0.0               //r_rps
      0.0               //north position ft
      0.0               //east position ft
      params.alt_ft     //alt_ft
      tgear(S(1))       //power_perc
     ];

controls.throttle = S(1);
controls.elev_deg = S(2);
controls.ail_deg = 0.0;
controls.rudder_deg = 0.0;
function xd = f16_model(t,X)
    [xd] = eqm(t, X, controls, params);
endfunction
function y = elev_step(t)
    if(t<0.5) then
        y = S(2);
    elseif (t>=0.5 && t<=0.53)
        y = S(2) - 1/0.03*(t-0.5);
    else
        y = S(2)-1;
    end
endfunction
t = 0:0.001:3;
controls.elev_deg = elev_step;
y = ode(X0, t(1), t, f16_model);

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
for i=1:length(t)
    [xd,outputs] = eqm(t(i), y(:,i), controls, params);
    nz_g(i) = outputs.nz_g;
    nx_g(i) = outputs.nx_g;
    nzs_g(i) = nx_g(i)*sin(y(2,i))+nz_g(i)*cos(y(2,i));
    mach(i) = outputs.mach;
    thrust_pound(i) = outputs.thrust_pound*sin(y(5,i));
    for j=1:len_acc
        az_acc_g(i,j) = nzs_g(i) + xd(8)*x_acc_ft(j)/params.g0_ftps2;
    end
end

/* Linearizing... */
disp('Linearizing...');
l_arm_ft = 15;
X0 = [
      params.VT_ftps    //VT_fps
      S(3)              //alpha_rad
      S(3)              //theta_rad
      0.0               //q_rps
      params.alt_ft     //alt_ft
      tgear(S(1))       //power_perc
     ];
U0 = [
     S(1) //throttle
     S(2) //elevator
     ];
function [y,xd] = sim_f16(X,U)
    controls.throttle = U(1);
    controls.elev_deg = U(2);
    controls.ail_deg = 0.0;
    controls.rudder_deg = 0.0;
    X_full = zeros(20,0);
    X_full(1) = X(1);
    X_full(2) = X(2);
    X_full(5) = X(3);
    X_full(8) = X(4);
    X_full(12)= X(5);
    X_full(13)= X(6);
    [xd_full,outputs] = eqm(0, X_full, controls, params);
    xd = xd_full([1 2 5 8 12 13]);
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
[A,B,C,D] = lin(sim_f16, X0, U0);
ss = syslin("c", A, B, C, D);
disp('Simulating linear model...');
function u = elev_step_lin(t)
    u = elev_step(t)-S(2)
endfunction
[y,x] = csim(elev_step_lin,t,ss(9,2));
