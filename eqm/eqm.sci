exec('atmosphere/atmosphere.sci');
exec('atmosphere/atmos_constants.sci');

function [mach, Q_Pa] = airdata(vt_mps, alt_m)
    [T_K, p_Pa, rho_kgpm3] = atmosphere(alt_m,0);
    mach = vt_mps/sqrt(1.4*atmos.R*T_K);
    Q_Pa = 0.5*rho_kgpm3*vt_mps^2;
endfunction

function eqm(t, X, XD)
    //F-16 model from Stevens And Lewis,second edition, pg 184
    mass = struct(AXX,9496.0, AYY, 55814.0, AZZ, 63100.0, AXZ, 982.0);
    mass.AXZ2 = mass.AXX**2;
    mass.XPQ = mass.AXZ*(mass.AXX-mass.AYY+mass.AZZ);
    mass.GAM = mass.AXX*mass.AZZ-mass.AXZ**2;
    mass.XQR = mass.AZZ*(mass.AZZ-mass.AYY)+mass.AXZ2;
    mass.ZPQ = (mass.AXX-mass.AYY)*mass.AXX+mass.AXZ2;
    mass.YPR = mass.AZZ - mass.AXX;
    mass.weight_pound = 20490.446;
    g0_ftps2 = 32.17;
    mass.mass_slug = mass.weight_pound/g0_ftps2;
    
    geom = struct(wing_ft2, 300, wingspan_ft, 30, chord_ft, 11.32, xcg_mac, 0.35);
    geom.engmomenthx_slugft2ps = 160;
    
    rad2deg = 57.29578;
    ft2m = 0.3048;
    kn2mps = 0.514444;
    
    //Assign state & control variables
    VT = X(1);
    alpha_deg = X(2)*rad2deg;
    beta_deg = X(3)*rad2deg;
    phi_deg = X(4);
    theta_deg = X(5);
    psi_deg = X(6);
    p_dps = X(7);
    q_dps = X(8);
    r_dps = X(9);
    alt_ft = X(12);
    pow_pound = X(13);
    
    //Air data computer and engine model
    [mach, Q_Pa] = airdata(VT*kn2mps, alt_ft*ft2m);
endfunction
