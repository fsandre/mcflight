exec('trim/trim_f16.sci');
exec('eqm/params_f16.sci');
params = load_f16();
params.xcg = 0.35;
params.VT_ftps = 502;
params.alt_ft = 0;
X0 = [
   502.
   0.2391101
   0.0005096
   1.3665928
   0.0500909
   0.
   0.
   0.
   0.
   0.
   0.
   0.
   64.1323
   ];
controls.throttle = 0.835;
controls.elev_deg = -1.48;
controls.ail_deg = 0.0954;
controls.rudder_deg = -0.411;
function xd = f16_model(t,X)
    [xd] = eqm(t, X, controls, params);
endfunction
t = 0:0.001:180;
disp('Simulating...');
y = ode(X0, t(1), t, f16_model);
/*disp('Calculating further outputs...');
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
end*/
