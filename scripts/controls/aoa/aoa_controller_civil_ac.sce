/*
 * Pitch axis stability augmentation example
 */
clear
exec('atmosphere/atmosphere.sci');
exec('atmosphere/atmos_constants.sci');
//exec('eqm/engine_f16.sci');
//exec('eqm/aerodata_f16.sci');

function [mach, Q_Pa] = airdata(vt_mps, alt_m)
    [T_K, p_Pa, rho_kgpm3] = atmosphere(alt_m,0);
    mach = vt_mps/sqrt(1.4*atmos.R*T_K);
    Q_Pa = 0.5*rho_kgpm3*vt_mps^2;
endfunction

function [y, xd] = sim_transport_ac_lewis(x, u, params)
    s = 2170.0;
    cbar = 17.5;
    mass = 5000;
    Iyy = 4.1e6;
    tstat = 6e4;
    dtdv = -38;
    ze = 2.0;
    cdcls = 0.042;
    cla = .085; //deg
    cma = -.022; // deg
    cmde = -.016; // deg
    cmq = -16; // rad
    cmadot = -6; // rad
    cladot = 0.0; // rad
    rtod = 57.29578;
    gd = 32.17;
    thtl = u(1);
    elev = u(2);
    xcg = params.xcg;
    land = (params.is_clean_conf==0);
    vt = x(1);
    alpha = rtod*x(2); //deg
    theta = x(3);
    q = x(4);
    h = x(5);
    ft2m = 0.3048;
    [mach, Q_Pa] = airdata(vt*ft2m, h*ft2m)
    qbar = Q_Pa*0.0208854;
    qs = qbar*s;
    salp = sin(x(2));
    calp = cos(x(2));
    gam = theta - x(2);
    sgam = sin(gam);
    cgam = cos(gam);
    if (~land) then // clean configuration
        cl0 =.20;
        cd0 =.016;
        cm0 =.05;
        dcdg=0.0;
        dcmg=0.0
    else
        cl0 = 1.0;
        cd0=.08;
        cm0=-.20;
        dcdg=.02;
        dcmg=-.05;
    end
    thr = (tstat + vt*dtdv)*max(thtl,0);
    cl = cl0 + cla*alpha;
    cm = dcmg + cm0 + cma*alpha + cmde*elev + cl*(xcg-.25);
    cdd = dcdg + cd0 + cdcls*cl*cl;
    
    xd(1) = (thr*calp - qs*cdd)/mass - gd*sgam; //Vt
    xd(2) = (-thr*salp - qs*cl + mass*(vt*q + gd*cgam))/(mass*vt + qs*cladot); //alpha
    xd(3) = q; // theta
    d = .5*cbar*(cmq*q + cmadot*xd(2))/vt;
    xd(4) = (qs*cbar*(cm + d) + thr*ze)/Iyy; //q
    xd(5) = vt*sgam; // altitude
    xd(6) = vt*cgam; // range
    
    y(1:4) = x(1:4);
    nxs = (thr*calp - qs*cdd)/mass/gd;
    nzs = (-thr*salp - qs*cl)/mass/gd;
    y(5) = nxs*calp - nzs*salp;
    y(6) = 0;
    y(7) = -(nxs*salp + nzs*calp);
endfunction

throttle_trim_u = 0.1845;
elev_trim_deg = -9.2184;
V_trim_fps = 250;
alpha_trim_rad = 0.16192;
theta_trim_rad = 0.16192;
q_trim_rps = 0;
params.xcg = 0.25;
params.is_clean_conf = 1;

X0 = [
    V_trim_fps
    alpha_trim_rad
    theta_trim_rad
    q_trim_rps
    ];
U0 = [
    throttle_trim_u
    elev_trim_deg
];

function [y, xd] = sim_ac_to_lin(x, u)
    x(5) = 0;
    x(6) = 0;
    [y_full, xd_full] = sim_transport_ac_lewis(x, u, params)
    xd(1:4) = xd_full(1:4);
    y = y_full;
end
// U = [tht elev], X=[V alpha theta q]
[A_stab, B_stab, C_stab, D_stab] = lin(sim_ac_to_lin, X0, U0);
[Y0, XD0] = sim_ac_to_lin(X0, U0);
// Including actuator as a simple-lag filter 
// with time constant tau_act = 1/20.2
// and an alpha filter tau_alpha = 0.1 s
tau_act = 1/20.2;
tau_alpha = 0.1;
omega_tas_cf = 1;
zeta_tas_cf = 1;
omega_alpha_cf = 5;
zeta_alpha_cf = 1;
alpha_trim_rad = X0(2);
theta_trim_rad = X0(3);
q_trim_rps = X0(4);
nxb_trim_g = Y0(5);
nyb_trim_g = Y0(6);
nzb_trim_g = Y0(7);
nxs_trim_g = nxb_trim_g*cos(alpha_trim_rad) - nzb_trim_g*sin(alpha_trim_rad); 
nzs_trim_g = nzb_trim_g*cos(alpha_trim_rad) + nxb_trim_g*sin(alpha_trim_rad);
g0_fps2 = atmos.g0_mps2/0.3048;

// Including Vdot
A_stab_2 = A_stab^2;
AB_stab = A_stab*B_stab;
A_vdot = [
          A_stab           zeros(4,1)
          A_stab_2(1,:)   0
         ];
B_vdot = [
          B_stab
          AB_stab(1,:)
         ];
C_vdot = eye(5,5);
D_vdot = zeros(5,2);

// Including alpha integral
A_vdot_alpha_int = [
                    A_vdot    zeros(5,1)
                    0 1 0 0 0 0
                    ];
B_vdot_alpha_int = [
                    B_vdot
                    0 0
                    ];
C_vdot_alpha_int = eye(6,6);
D_vdot_alpha_int = zeros(6,2);
//C_vdot_alpha_int = [1 0 0 0 0 0
//                    0 1 0 0 0 0];
//D_vdot_alpha_int = zeros(2,2);
ss_lqr_full = syslin("c", A_vdot_alpha_int, B_vdot_alpha_int, C_vdot_alpha_int, D_vdot_alpha_int);
ss_lqr_sp = syslin("c", A_vdot_alpha_int([1 2 4 6],[1 2 4 6]), B_vdot_alpha_int([1 2 4 6],2), eye(4,4), zeros(4,1));
Q = [
     .01    0    0    0    0    0 //Vt
     0    2    0    0    0    0 //alpha
     0    0    .01    0    0    0 //theta
     0    0    0    2    0    0 //q
     0    0    0    0    1    0 //Vdot
     0    0    0    0    0    1e3 //alphaInt
    ];
R = [
      1e2     0
      0     1
     ];
//Kc = lqr(ss_lqr_full, Q, R);

// Short period in AoA
Q = [
     1e-2  0  0  0 //Vt
     0  1  0  0 //alpha
     0  0  1  0 //q
     0  0  0  1e1 //alphaInt
      ];
R = [1e-2];

Kc_sp = lqr(ss_lqr_sp, Q, R);
// Gains
Kint = 0.551; //-Kc(2,6)/180*%pi;
KP = 0.1098;//Kc(2,2)/180*%pi;
Kq = 0.6;//Kc(2,4)/180*%pi;;
KD = 0*0.6;
Ktasdot = 0.009//-Kc(2,5)/180*%pi;;
KV = 0.007;
Ktheta = 0*0.8;

//Very good response with V and theta feedback
//Kint = 0.551; //-Kc(2,6)/180*%pi;
//KP = 0.1098;//Kc(2,2)/180*%pi;
//Kq = 0.6;//Kc(2,4)/180*%pi;;
//KD = 0*0.6;
//Ktasdot = 0*0.09//-Kc(2,5)/180*%pi;;
//KV = 0.01;
//Ktheta = 0.8;
printf("KI= %.4f, KP= %.4f, Kq=%.4f, Ktasdot= %.4f", Kint, KP, Kq, Ktasdot);
