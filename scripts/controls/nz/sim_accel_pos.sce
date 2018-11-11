/*
 * Checking accelerometer postion and NMP zero in load factor response
 */
clear
exec('eqm/params_f16.sci');
params = load_f16();
exec('controls/nz/trim_v502_alt0_xcg35_level.sce');

/* Linearizing... */
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

l_arm_ft = 15;//long_deriv_body.Zelev/long_deriv_body.Melev;
disp('Including nz (stability) on different position as state-space output: '+string(l_arm_ft)+' ft.');
// The sign of Nz is being considered positive up here
C_body(9,:) = C_body(8,:) + l_arm_ft/params.g0_ftps2*A_body(4,:);
D_body(9,:) = D_body(8,:) + l_arm_ft/params.g0_ftps2*B_body(4,:);

//Including actuator model of simple-lag tau=1/20.2
tau_act = 1/20.2;
A_aug = [A_body     -B_body(:,2)
         zeros(1,4) -1/tau_act   ];
B_aug = [B_body(:,1) zeros(4,1)
         0     1/tau_act];
C_aug = [C_body zeros(9,1)];
D_aug = D_body;
ss_act = syslin("c", A_aug, B_aug, C_aug, D_aug);

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
[y_body,x_body] = csim(elev_step_lin, t, ss_act(:,2));



evans(ss_act(5,2)*180/%pi,10); //root-locus of pitch-rate response
//cl_pitch_rate = ss_body(5,2)/.(-22);//closing loop in pitch
