exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');
params = load_f16();
params.xcg = .35;
params.coordinated_turn = 0;
params.turn_rate_rps = 0.0;
params.roll_rate_rps = 0.0;
params.pitch_rate_rps = 0.0;
params.phi_rad = 0.0;
params.gamma_rad = 0.0;
params.stability_axis_roll = 0;
params.VT_ftps = 200;
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
    if(t<=0.4) then
        y = S(2);
    elseif (t>0.4 && t<0.5)
        y = S(2);//(t-0.4);
    else
        y = S(2) + 0.1;
    end
endfunction
function y = ail_step(t)
    if(t<=0.4) then
        y = 0.0;
    elseif (t>0.4 && t<0.5)
        y = 0.0;//(t-0.4);
    else
        y = 0.1;
    end
endfunction
t = 0:0.1:5;
//controls.elev_deg = elev_step;
controls.ail_deg = ail_step;
disp('Simulating...');
y = ode(X0, t(1), t, f16_model);

disp('Calculating further outputs...');
nz_g = 0*t;
nx_g = 0*t;
nzs_g = 0*t;
mach = 0*t;
thrust_pound = 0*t;
for i=1:length(t)
    [xd,outputs] = eqm(t(i), y(:,i), controls, params);
    nz_g(i) = outputs.nz_g;
    nx_g(i) = outputs.nx_g;
    nzs_g(i) = nx_g(i)*sin(y(2,i))+nz_g(i)*cos(y(2,i));
    mach(i) = outputs.mach;
    thrust_pound(i) = outputs.thrust_pound*sin(y(5,i));
end
