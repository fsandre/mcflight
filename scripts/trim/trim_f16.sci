exec('eqm/engine_f16.sci');
exec('eqm/eqm.sci');
exec('eqm/params_f16.sci');

function [X, controls, params] = trim_straight_level(V_ftps, alt_ft, xcg)
    [out, inp] = argn(0);
    if inp<3 then
        xcg = .35;
    end
    if inp<2 then
        alt_ft = 0.0;
    end
    params = load_f16();
    params.xcg = xcg;
    params.coordinated_turn = 0;
    params.turn_rate_rps = 0.0;
    params.roll_rate_rps = 0.0;
    params.pitch_rate_rps = 0.0;
    params.phi_rad = 0.0;
    params.gamma_rad = 0.0;
    params.stability_axis_roll = 0;
    params.VT_ftps = V_ftps;
    params.alt_ft = alt_ft;
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
    
    X = [
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
endfunction

function [X, controls, params] = trim_coordinated_turn(V_ftps, alt_ft, turn_rate_rps, gamma_rad, xcg)
    [out, inp] = argn(0);
    if inp<5 then
        xcg = .35;
    end
    if inp<4 then
        gamma_rad = 0.0;
    end
    params = load_f16();
    params.xcg = xcg;
    params.coordinated_turn = 1;
    params.turn_rate_rps = turn_rate_rps;
    params.roll_rate_rps = 0.0;
    params.pitch_rate_rps = 0.0;
    params.phi_rad = 0.0;
    params.gamma_rad = gamma_rad;
    params.stability_axis_roll = 0;
    params.VT_ftps = V_ftps;
    params.alt_ft = alt_ft;
    function y = costf16(x)
        y = cost_trim_f16(x,params);
    endfunction
    S0 = [
         .1   //(1)throttle 0-1
         0.0  //(2)elev_deg
         0.0  //(3)alpha_rad
         0.0  //(4)ail_deg
         0.0  //(5)rudder_deg
         0.0  //(6)beta_rad
         ];
    
    S = fminsearch(costf16, S0);
    X = [
          params.VT_ftps    //VT_fps
          S(3)              //alpha_rad
          S(6)              //beta_rad
          0.0               //phi_rad
          0.0               //theta_rad
          0.0               //psi_rad
          0.0               //p_rps
          0.0               //q_rps
          0.0               //r_rps
          0.0               //north position ft
          0.0               //east position ft
          params.alt_ft     //alt_ft
          tgear(S(1))       //power_perc
         ];
    X = trim_constraint_f16(X, params);
    controls.throttle = S(1);
    controls.elev_deg = S(2);
    controls.ail_deg = S(4);
    controls.rudder_deg = S(5);
endfunction

function y = cost_trim_f16(S, params)
    X = zeros(13,1);
    controls.throttle = S(1);
    controls.elev_deg = S(2);
    X(2) = S(3);
    if(length(S)>3) then
        controls.ail_deg = S(4);
        controls.rudder_deg = S(5);
        X(3) = S(6);
    else
        controls.ail_deg = 0.0;
        controls.rudder_deg = 0.0;
        X(3) = 0.0;
    end
    X(13) = tgear(controls.throttle);
    X = trim_constraint_f16(X, params);
    XD = eqm(0, X, controls, params);
    y = XD(1)^2 + 100*(XD(2)^2 + XD(3)^2) + 10*(XD(7)^2 + XD(8)^2 + XD(9)^2);
endfunction

function [X_new] = trim_constraint_f16(X, params)
    X(1) = params.VT_ftps;
    X(12) = params.alt_ft;
    cos_alpha = cos(X(2));
    sin_alpha = sin(X(2));
    tan_alpha = sin_alpha/cos_alpha;
    cos_beta = cos(X(3));
    sin_beta = sin(X(3));
    if(params.coordinated_turn) then
        sin_gamma = sin(params.gamma_rad);
        centripetal_acc = params.turn_rate_rps*params.VT_ftps/params.g0_ftps2;
        a = 1 - centripetal_acc*tan_alpha*sin_beta;
        b = sin_gamma/cos_beta;
        c = 1 + (centripetal_acc^2)*cos_beta^2;
        //phi_rad, Stevens&Lewis 2nd edition, eq 3.6-5
        X(4) = atan(centripetal_acc*(cos_beta/cos_alpha)*((a-b^2) + b*tan_alpha*sqrt(c*(1-b^2) + (centripetal_acc^2)*sin_beta^2))/(a^2 - (b^2)*(1 + c*tan_alpha^2)));
        //theta_rad, Stevens&Lewis 2nd edition eq 3.6-3
        a = cos_alpha*cos_beta;
        b = sin(X(4))*sin_beta + cos(X(4))*sin_alpha*cos_beta; 
        X(5) = atan((a*b + sin_gamma*sqrt(a^2 - sin_gamma^2 + b^2))/(a^2 - sin_gamma^2));
    elseif (params.turn_rate_rps <> 0) then
        //skidding turn logic
    else //non-turning flight
        X(4) = params.phi_rad;
        alpha_rad = X(2);
        if (params.phi_rad <> 0.0) then 
            alpha_rad = -X(2);
        end
        if (params.gamma_rad <> 0.0) then
            sin_gamma_over_cos_beta = sin(params.gamma_rad)/cos_beta;
            X(5) = alpha_rad + atan(sin_gamma_over_cos_beta/sqrt(1.0-sin_gamma_over_cos_beta*sin_gamma_over_cos_beta));
        else
            X(5) = alpha_rad;
        end
        X(7) = params.roll_rate_rps;
        X(8) = params.pitch_rate_rps;
        if(params.stability_axis_roll)
            X(9) = roll_rate_rps*sin_alpha/cos_alpha; // stability-axis roll
        else
            X(9) = 0.0;                               // body-axis roll
        end
    end
    X_new = X;
endfunction
