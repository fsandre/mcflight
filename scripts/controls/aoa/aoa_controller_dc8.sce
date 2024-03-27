/*
 * Pitch axis stability augmentation example
 */
clear
exec('atmosphere/atmosphere.sci');
exec('atmosphere/atmos_constants.sci');

// Mcruer(1997) DC-8 case 8001
g0_fps2 = 32.17404;
h_ft = 0;
Mach = 0.219;
V_trim_fps = 243.5;
weight_lb = 190000;
mass_slug = 5900;
Ixx = 3.09e6;
Iyy = 2.94e6;
Izz = 5.58e6;
Ixz = 2.8e5;
xcg_mac = 0.15;
theta_trim_rad = 0;
u_trim_fps = 243.5;
w_trim_fps = 0;
flap_trim_deg = 35;
alpha_trim_rad = 0;
q_trim_rps = 0;
elev_trim_deg = 0;

nxb_trim_g = (q_trim_rps*w_trim_fps + g0_fps2*sin(theta_trim_rad))/g0_fps2;
nyb_trim_g = 0;
nzb_trim_g = (q_trim_rps*u_trim_fps + g0_fps2*cos(theta_trim_rad))/g0_fps2; // positive up
nzs_trim_g = nzb_trim_g*cos(alpha_trim_rad) + nxb_trim_g*sin(alpha_trim_rad);
nxs_trim_g = nxb_trim_g*cos(alpha_trim_rad) - nzb_trim_g*sin(alpha_trim_rad); 

X0 = [
    V_trim_fps
    alpha_trim_rad
    theta_trim_rad
    q_trim_rps
    ];
U0 = [
    elev_trim_deg
];
T_u = -0.000595;
X_u = -0.0291; //[1/s]
X_w = 0.0629;
X_wdot = 0;
X_de = 0;
Z_u = -0.2506;
Z_wdot = 0;
Z_w = -0.6277;
Z_de = -10.19;
M_u = -0.0000077;
M_wdot = -0.001068;
M_w = -0.0087;
M_q = -0.7924;
M_de = -1.35;
X_q = 0;
Z_q = 0;

x_u = X_u + X_wdot*Z_u/(1 - Z_wdot);
z_u = Z_u;
m_u = M_u + Z_u*M_wdot/(1 - Z_wdot);
x_w = X_w + X_wdot*Z_w/(1 - Z_wdot);
z_w = Z_w/(1 - Z_wdot);
m_w = M_w + Z_w*M_wdot/(1 - Z_wdot);
x_q = X_q - w_trim_fps + (Z_q + u_trim_fps)*X_wdot/(1 - Z_wdot);
z_q = (Z_q + u_trim_fps)/(1 - Z_wdot);
m_q = M_q + (Z_q + u_trim_fps)*M_wdot/(1 - Z_wdot);
x_theta = -g0_fps2*cos(theta_trim_rad) - X_wdot*g0_fps2*sin(theta_trim_rad)/(1 - Z_wdot);
z_theta = -g0_fps2*sin(theta_trim_rad)/(1 - Z_wdot);
m_theta = -M_wdot*g0_fps2*sin(theta_trim_rad)/(1 - Z_wdot);

x_de = X_de + X_wdot*Z_de/(1 - Z_wdot);
z_de = Z_de/(1 - Z_wdot);
m_de = M_de + M_wdot*Z_de/(1 - Z_wdot);

A_body = [
         x_u    x_w    x_theta    x_q
         z_u    z_w    z_theta    z_q
         0      0      0          1
         m_u    m_w    m_theta    m_q   
         ];

B_body = [
          x_de
          z_de
          0
          m_de
          ];
C_body = [1    0    0    0
          0    1    0    0
          0    0    1    0
          0    0    0    1
          (A_body(1,:) + [0, q_trim_rps, g0_fps2, w_trim_fps])/g0_fps2 //nx g
          0    0    0    0//ny g
          -(A_body(2,:) - [-q_trim_rps, 0, g0_fps2*sin(theta_trim_rad), -u_trim_fps])/g0_fps2 // nz g positive up
          ];
D_body = [
         0
         0
         0
         0
         B_body(1,:)
         0
         -B_body(2,:)
         ];
         
T_stab_body = [
               cos(alpha_trim_rad)  -V_trim_fps*sin(alpha_trim_rad)    0    0
               sin(alpha_trim_rad)   V_trim_fps*cos(alpha_trim_rad)    0    0
               0                                 0                     1    0
               0                                 0                     0    1
               ];
               
A_stab = T_stab_body\A_body*T_stab_body;
B_stab = T_stab_body\B_body;
C_stab = C_body*T_stab_body;
D_stab = D_body;

// Changing output to V and alpha
C_stab(1,:) = [1 0 0 0];
D_stab(1,:) = [0];
C_stab(2,:) = [0 1 0 0];
D_stab(2,:) = [0];

// and an alpha filter tau_alpha = 0.1 s
tau_act = 1/20.2;
tau_alpha = 0.1;
omega_tas_cf = 1;
zeta_tas_cf = 1;
omega_alpha_cf = 5;
zeta_alpha_cf = 1;

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
D_vdot = zeros(5,1);

// Including alpha integral
A_vdot_alpha_int = [
                    A_vdot    zeros(5,1)
                    0 1 0 0 0 0
                    ];
B_vdot_alpha_int = [
                    B_vdot
                    0
                    ];
C_vdot_alpha_int = eye(6,6);
D_vdot_alpha_int = zeros(6,1);
//C_vdot_alpha_int = [1 0 0 0 0 0
//                    0 1 0 0 0 0];
//D_vdot_alpha_int = zeros(2,2);
ss_lqr_full = syslin("c", A_vdot_alpha_int, B_vdot_alpha_int, C_vdot_alpha_int, D_vdot_alpha_int);
//t = 0:.01:5;
//u = 0*t + 1;
//y = csim(%pi/180*u, t, ss_lqr_full(:,1));
//figure;plot(t(1:(size(t,2)-1)), diff(y(1,:))./diff(t), t, y(5,:), 'r--'); title('Vdot');
//figure;plot(t(1:(size(t,2)-1)), diff(y(6,:))./diff(t), t, y(2,:), 'r--'); title('Alpha');
ss_lqr_sp = syslin("c", A_vdot_alpha_int([1 2 4 6],[1 2 4 6]), B_vdot_alpha_int([1 2 4 6],1), eye(4,4), zeros(4,1));
Q = [
     .1    0    0    0    0    0 //Vt
     0    1    0    0    0    0 //alpha
     0    0    1    0    0    0 //theta
     0    0    0    1    0    0 //q
     0    0    0    0    1    0 //Vdot
     0    0    0    0    0    1e4 //alphaInt
    ];
R = [
      1
     ];
Kc = lqr(ss_lqr_full, Q, R);
Ksp = lqr(ss_lqr_sp, Q([1 2 4 6],[1 2 4 6]), 1);

//Including fake throttle input
B_stab = [zeros(4,1) B_stab]*%pi/180;
D_stab = [zeros(7,1) D_stab]*%pi/180;

// LQR Gains
//Kc = Kc/180*%pi;
//Kint = 1.7453;//Kc(1,6);
//KP = 0.5335;//Kc(1,2);
//Kq = 0.1018;//Kc(1,4);
//Ktasdot = 0.0;//-Kc(1,5);
//Ktheta = 0.0;//-Kc(1,3);
//KV = 0.05;//-Kc(1,1);

//Forced gains
Kint = 0.9;
KP = 0.2;
Kq = 1.9;
KD = 0*1.7;
Ktasdot = 0*0.03;
Ktheta = 1.2;
KV = 0.01;
printf("KI= %.4f, KP= %.4f, Kq=%.4f, Ktasdot= %.4f, KV=%.4f, Ktheta=%.4f", Kint, KP, Kq, Ktasdot,KV,Ktheta);

Kc = [-KV, -KP, Ktheta, Kq, -Ktasdot, Kint];
ss_cl_state = ss_lqr_full /. -Kc;
ss_cl_state.B = [zeros(5,1); -1]; // putting alpha as input command
