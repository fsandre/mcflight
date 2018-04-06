exec('trim/trim_f16.sci');
exec('eqm/engine_f16.sci');
params.xcg = .35;
params.coordinated_turn = 0;
params.turn_rate_rps = 0.0;
params.roll_rate_rps = 0.0;
params.pitch_rate_rps = 0.0;
params.phi_rad = 0.0;
params.gamma_rad = 0.0;
params.stability_axis_roll = 0;
params.VT_ftps = 350;
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

X0 = [
      params.VT_ftps    //VT_fps
      S(3)              //alpha_rad
      S(3)              //theta_rad
      0.0               //q_rps
      tgear(S(1))       //power_perc
     ];

U0 = [
     S(1)
     S(2)
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
    X_full(13)= X(5);
    [xd_full,outputs] = eqm(0, X_full, controls, params);
    xd = xd_full([1 2 5 8 13]);
    y = [
        outputs.nz_g;
        outputs.ny_g;
        outputs.nx_g;
        outputs.Q_lbfpft2;
        outputs.mach;
        outputs.q_rps;
        outputs.alpha_deg;
        outputs.nx_g*sin(X(2)) + outputs.nz_g*cos(X(2));
        ];
endfunction

[A,B,C,D] = lin(sim_f16, X0, U0);
ss = syslin("c", A, B, C, D);

function y = elev_step_lin(t)
    if(t<0.5) then
        y = 0.0;
    elseif (t>=0.5 && t<=0.53)
        y = - 1/0.03*(t-0.5);
    else
        y = -1;
    end
endfunction
t = 0:0.001:3;
[y,x] = csim(elev_step_lin,t,ss(8,2));

//Removing zero with all pass filter
[z,p,k] = ss2zp(ss(8,2));
tf_no_zero = zp2tf([-z(1);z],[z(1);p],-k,"c");
[y_no_zero,x_no_zero] = csim(elev_step_lin,t,tf_no_zero);
