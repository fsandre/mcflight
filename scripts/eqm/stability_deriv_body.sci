function [long, lat] = stability_deriv_body(eqm_fun, X0, controls, params, dX)
    [out, inp] = argn(0);
    if(inp<5) then
        onedeg_rad = 1/180*%pi;
        dX = [
              .1 //(1)u_ftps
              .1 //(2)v_ftps
              .1 //(3)w_ftps
              onedeg_rad //(4)phi_rad
              onedeg_rad //(5)theta_rad
              onedeg_rad //(6)psi_rad
              onedeg_rad //(7)p_rps
              onedeg_rad //(8)q_rps
              onedeg_rad //(9)r_rps
              .1 //(10)north position ft
              .1 //(11)east position ft
              .1 //(12)alt_ft
              .1 //(13)power_perc
             ];
    end
    rad2deg = 180/%pi;
    
    // u derivatives
    X_up = X0;
    X_up(1) = X_up(1) + dX(1)/2;
    X_down = X0;
    X_down(1) = X_down(1) - dX(1)/2;
    [XD_up, out_up] = eqm_fun(0, X_up, controls, params);
    [XD_down, out_down] = eqm_fun(0, X_down, controls, params);
    long.CXu = (out_up.aero_forces(1) - out_down.aero_forces(1))./dX(1);
    long.CZu = (out_up.aero_forces(3) - out_down.aero_forces(3))./dX(1);
    long.CMu = (out_up.aero_moments(2) - out_down.aero_moments(2))./dX(1);

    // w derivatives
    X_up = X0;
    X_up(3) = X_up(3) + dX(3)/2;
    X_down = X0;
    X_down(3) = X_down(3) - dX(3)/2;
    [XD_up, out_up] = eqm_fun(0, X_up, controls, params);
    [XD_down, out_down] = eqm_fun(0, X_down, controls, params);
    long.CXw = (out_up.aero_forces(1) - out_down.aero_forces(1))./dX(3);
    long.CZw = (out_up.aero_forces(3) - out_down.aero_forces(3))./dX(3);
    long.CMw = (out_up.aero_moments(2) - out_down.aero_moments(2))./dX(3);
    long.CXwdot = (out_up.aero_forces(1) - out_down.aero_forces(1))./(XD_up(3)-XD_down(3));
    long.CZwdot = (out_up.aero_forces(3) - out_down.aero_forces(3))./(XD_up(3)-XD_down(3));
    long.CMwdot = (out_up.aero_moments(1) - out_down.aero_moments(1))./(XD_up(3)-XD_down(3));
    
    long.Thrust_v = (out_up.thrust_pound - out_down.thrust_pound)./dX(1);
    
    // Pitch derivatives
    X_up = X0;
    X_up(8) = X_up(8) + dX(8)/2;
    X_down = X0;
    X_down(8) = X_down(8) - dX(8)/2;
    [XD_up, out_up] = eqm_fun(0, X_up, controls, params);
    [XD_down, out_down] = eqm_fun(0, X_down, controls, params);
    long.CXq = (out_up.aero_forces(1) - out_down.aero_forces(1))./dX(8);
    long.CZq = (out_up.aero_forces(3) - out_down.aero_forces(3))./dX(8);
    long.CMq = (out_up.aero_moments(2) - out_down.aero_moments(2))./dX(8);
    
    // Elevator
    X = X0;
    dElev = 1;
    c_down = controls;
    c_down.elev_deg = controls.elev_deg - dElev/2;
    c_up = controls
    c_up.elev_deg = controls.elev_deg + dElev/2;
    
    [XD_up, out_up] = eqm_fun(0, X0, c_up, params);
    [XD_down, out_down] = eqm_fun(0, X0, c_down, params);
    long.CZelev = (out_up.aero_forces(3) - out_down.aero_forces(3))./dElev;
    long.CXelev = (out_up.aero_forces(1) - out_down.aero_forces(1))./dElev;
    long.CMelev = (out_up.aero_moments(2) - out_down.aero_moments(2))./dElev;
    
    // Throttle
    X = X0;
    dThrottle = 1;
    X(13) = X0(13) - dThrottle/2;
    [XD_down, out_down] = eqm_fun(0, X, controls, params);
    X(13) = X0(13) + dThrottle/2;
    [XD_up, out_up] = eqm_fun(0, X, controls, params);
    long.Thrust_throttle = (out_up.thrust_pound - out_down.thrust_pound)./dThrottle;
    long.CMthrottle = (out_up.aero_moments(2) - out_down.aero_forces(2))./dThrottle;
    Q_lbfpft2 = out_down.Q_lbfpft2;
    
    // Dimensional derivatives
    qS = Q_lbfpft2*params.geom.wing_ft2;
    c = params.geom.chord_ft;
    qSc = qS*params.geom.chord_ft;
    m = params.mass.mass_slug;
    Iyy = params.mass.AYY;

    long.Xu = qS/m/params.VT_ftps * long.CXu;
    long.Xw = qS/m/params.VT_ftps * long.CXw;
    long.Xwdot = qSc/m/(params.VT_ftps^2) * long.CXwdot;
    long.Xq = qSc/m/params.VT_ftps * long.CXq;
    long.Zu = qS/m/params.VT_ftps * long.CZu;
    long.Zw = qS/m/params.VT_ftps * long.CZw;
    long.Zwdot = qSc/m/(params.VT_ftps^2) * long.CZwdot;
    long.Zq = qSc/m/params.VT_ftps * long.CZq;
    long.Mu = qSc/Iyy/params.VT_ftps * long.CMu;
    long.Mw = qSc/Iyy/params.VT_ftps * long.CMw;
    long.Mwdot = qSc*c/m/(params.VT_ftps^2) * long.CMwdot;;
    long.Mq = qSc*c/Iyy/params.VT_ftps * long.CMq;
    long.Zelev = qS/m * long.CZelev;
    long.Xelev = qS/m * long.CXelev;
    long.Melev = qSc/Iyy * long.CMelev;
    long.XTv = long.Thrust_v/m;
    long.Xthrottle = long.Thrust_throttle/m;
    long.Zthrottle = 0.0;
    long.Mthrottle = qSc/Iyy * long.CMthrottle;
    // Body-axis matrices [ u w theta q ] Ref. Cook Flight Dynamics Principles, 2nd ed 2007
    long.xu = long.Xu + (long.Xwdot*long.Zu)/(1 - long.Zwdot);
    long.zu = long.Zu/(1-long.Zwdot);
    long.mu = long.Mu + (long.Zu*long.Mwdot)/(1-long.Zwdot);
    long.xw = long.Xw + (long.Xwdot*long.Zw)/(1-long.Zwdot);
    long.zw = long.Zw/(1-long.Zwdot);
    long.mw = long.Mw + (long.Zw*long.Mwdot)/(1-long.Zwdot);
    long.xq = (long.Xq - X0(3)) + (long.Zq + X0(1))*long.Xwdot/(1-long.Zwdot);
    long.zq = (long.Zq + X0(1))/(1-long.Zwdot);
    long.mq = long.Mq + (long.Zq + X0(1))*long.Mwdot/(1-long.Zwdot);
    long.xtheta = -params.g0_ftps2*cos(X0(5)) - (long.Xwdot*params.g0_ftps2*sin(X0(5)))/(1-long.Zwdot);
    long.ztheta = -(params.g0_ftps2*sin(X0(5)))/(1-long.Zwdot);
    long.mtheta = -long.Mwdot*params.g0_ftps2*sin(X0(5))/(1-long.Zwdot);
    
    long.xelev = long.Xelev + long.Xwdot*long.Zelev/(1-long.Zwdot);
    long.zelev = long.Zelev/(1-long.Zwdot);
    long.melev = long.Melev + long.Mwdot*long.Zelev/(1-long.Zwdot);
    long.xthrottle = long.Xthrottle + long.Xwdot*long.Zthrottle/(1-long.Zwdot);
    long.zthrottle = long.Zthrottle/(1-long.Zwdot);
    long.mthrottle = long.Mthrottle + long.Mwdot*long.Zthrottle/(1-long.Zwdot);
    
    long.A = [long.xu    long.xw    long.xtheta    long.xq
              long.zu    long.zw    long.ztheta    long.zq
              0                     0                     0                         1
              long.mu    long.mw    long.mtheta    long.mq];
    long.B = [long.xelev   long.xthrottle
              long.zelev   long.zthrottle
              0                       0
              long.melev   long.mthrottle];
              
    long.state_labels = ['u' 'w' 'theta' 'q'];
    long.input_labels = ['elev' 'throttle'];
    // TODO: lateral derivatives
    lat = struct();
endfunction
