exec('atmosphere/atmosphere.sci');
exec('atmosphere/atmos_constants.sci');
//exec('eqm/engine_f16.sci');
//exec('eqm/aerodata_f16.sci');

function [mach, Q_Pa] = airdata(vt_mps, alt_m)
    [T_K, p_Pa, rho_kgpm3] = atmosphere(alt_m,0);
    mach = vt_mps/sqrt(1.4*atmos.R*T_K);
    Q_Pa = 0.5*rho_kgpm3*vt_mps^2;
endfunction

// Utility to allow functions as control inputs 
function v = get_control_value(t, value)
    if type(value)==13 then
        v = value(t);
    else
        v = value;
    end
endfunction

function [XD,outputs] = eqm(t, X, controls, params)
    // F-16 model from Stevens And Lewis,second edition, pg 184
    mass = params.mass;
    geom = params.geom;
    
    g0_ftps2 = 32.17;
    rad2deg = 57.29578;
    ft2m = 0.3048;
    kn2mps = 0.514444;
    
    //Control variables
    throttle_u = get_control_value(t,controls.throttle);
    elev_deg = get_control_value(t,controls.elev_deg);
    ail_deg = get_control_value(t,controls.ail_deg);
    rudder_deg = get_control_value(t,controls.rudder_deg);
    
    // Assign state & control variables
    VT_ftps = X(1);
    alpha_deg = X(2)*rad2deg;
    beta_deg = X(3)*rad2deg;
    phi_rad = X(4);
    theta_rad = X(5);
    psi_rad = X(6);
    p_rps = X(7);
    q_rps = X(8);
    r_rps = X(9);
    alt_ft = X(12);
    pow = X(13);
    
    // Air data computer and engine model
    [mach, Q_Pa] = airdata(VT_ftps*ft2m, alt_ft*ft2m);
    Q_lbfpft2 = Q_Pa*0.0208854; //from Pascal to lbf/ft2
    
    // Engine model
    cpow = tgear(throttle_u);
    XD(13) = pdot(pow, cpow);
    thrust_pound = thrust(pow, alt_ft, mach);
    
    // Look-up tables and component buildup
    CXT = CX(alpha_deg, elev_deg);
    CYT = CY(beta_deg, ail_deg, rudder_deg);
    CZT = CZ(alpha_deg, beta_deg, elev_deg);
    dail = ail_deg/20.0;
    drdr = rudder_deg/30.0;
    CLT = CL(alpha_deg, beta_deg) + DLDA(alpha_deg, beta_deg)*dail + DLDR(alpha_deg, beta_deg)*drdr;
    CMT = CM(alpha_deg, elev_deg);
    CNT = CN(alpha_deg, beta_deg) + DNDA(alpha_deg, beta_deg)*dail + DNDR(alpha_deg, beta_deg)*drdr;
    
    // Add damping derivatives
    TVT = 0.5/VT_ftps;
    B2V = geom.wingspan_ft*TVT;
    CQ = geom.chord_ft*q_rps*TVT;
    D = aerodynamic_damp(alpha_deg);
    CXT = CXT + CQ*D(1);
    CYT = CYT + B2V*(D(2)*r_rps + D(3)*p_rps);
    CZT = CZT + CQ*D(4);
    CLT = CLT + B2V*(D(5)*r_rps + D(6)*p_rps);
    CMT = CMT + CQ*D(7) + CZT*(geom.xcgr_mac - params.xcg);
    CNT = CNT + B2V*(D(8)*r_rps + D(9)*p_rps) - CYT*(geom.xcgr_mac - params.xcg)*geom.chord_ft/geom.wingspan_ft;
    
    // Get ready for state equations
    cos_beta = cos(X(3));
    sin_theta = sin(theta_rad);
    cos_theta = cos(theta_rad);
    sin_phi = sin(phi_rad);
    cos_phi = cos(phi_rad);
    sin_psi = sin(psi_rad);
    cos_psi = cos(psi_rad);
    QS = Q_lbfpft2*geom.wing_ft2;
    QSb = QS*geom.wingspan_ft;
    g0_cos_theta = g0_ftps2*cos_theta;
    Q_sin_phi = q_rps*sin_phi;
    QS_over_mass = QS/mass.mass_slug;
    
    u_ftps = VT_ftps*cos(X(2))*cos_beta;
    v_ftps = VT_ftps*sin(X(3));
    w_ftps = VT_ftps*sin(X(2))*cos_beta;
    
    ax_ftps2 = (QS*CXT + thrust_pound)/mass.mass_slug;
    ay_ftps2 = QS_over_mass*CYT;
    az_ftps2 = QS_over_mass*CZT;
    
    // Force equations
    udot_ftps2 = r_rps*v_ftps - q_rps*w_ftps - g0_ftps2*sin_theta   + ax_ftps2;
    vdot_ftps2 = p_rps*w_ftps - r_rps*u_ftps + g0_cos_theta*sin_phi + ay_ftps2;
    wdot_ftps2 = q_rps*u_ftps - p_rps*v_ftps + g0_cos_theta*cos_phi + az_ftps2;
    u2_plus_w2 = u_ftps^2 + w_ftps^2;
    XD(1) = (u_ftps*udot_ftps2 + v_ftps*vdot_ftps2 + w_ftps*wdot_ftps2)/VT_ftps;
    XD(2) = (u_ftps*wdot_ftps2 - w_ftps*udot_ftps2) / u2_plus_w2;
    XD(3) = (VT_ftps*vdot_ftps2 - v_ftps*XD(1))*cos_beta / u2_plus_w2;
    
    // Kinematics
    XD(4) = p_rps + (sin_theta/cos_theta)*(Q_sin_phi + r_rps*cos_phi);
    XD(5) = q_rps*cos_phi - r_rps*sin_phi;
    XD(6) = (Q_sin_phi + r_rps*cos_phi)/cos_theta;
    
    // Moments
    roll_rps = QSb*CLT;
    pitch_rps = QS*geom.chord_ft*CMT;
    yaw_rps = QSb*CNT;
    p_q = p_rps*q_rps;
    q_r = q_rps*r_rps;
    q_hx = q_rps*geom.engmomenthx_slugft2ps;
    XD(7) = (mass.XPQ*p_q - mass.XQR*q_r + mass.AZZ*roll_rps + mass.AXZ*(yaw_rps + q_hx))/mass.GAM;
    XD(8) = (mass.YPR*p_rps*r_rps - mass.AXZ*(p_rps^2 - r_rps^2) + pitch_rps - r_rps*geom.engmomenthx_slugft2ps)/mass.AYY;
    XD(9) = (mass.ZPQ*p_q - mass.XPQ*q_r + mass.AXZ*roll_rps + mass.AXX*(yaw_rps + q_hx))/mass.GAM;
    
    // Navigation
    T1 = sin_phi*cos_phi;
    T2 = cos_phi*sin_theta;
    T3 = sin_phi*sin_psi;
    S1 = cos_theta*cos_psi;
    S2 = cos_theta*sin_psi;
    S3 = T1*sin_theta - cos_phi*sin_psi;
    S4 = T3*sin_theta + cos_phi*cos_psi;
    S5 = sin_phi*cos_theta;
    S6 = T2*cos_psi + T3;
    S7 = T2*sin_psi - T1;
    S8 = cos_phi*cos_theta;
    
    XD(10) = u_ftps*S1 + v_ftps*S3 + w_ftps*S6;        // North speed
    XD(11) = u_ftps*S2 + v_ftps*S4 + w_ftps*S7;        // East speed
    XD(12) = u_ftps*sin_theta - v_ftps*S5 - w_ftps*S8; // Vertical speed
    
    outputs.nz_g = -az_ftps2/g0_ftps2;
    outputs.ny_g = ay_ftps2/g0_ftps2;
    outputs.nx_g = ax_ftps2/g0_ftps2;
    outputs.Q_lbfpft2 = Q_lbfpft2;
    outputs.mach = mach;
    outputs.q_rps = q_rps;
    outputs.alpha_deg = alpha_deg;
    outputs.alt_ft = alt_ft;
    outputs.thrust_pound = thrust_pound;
    
endfunction
