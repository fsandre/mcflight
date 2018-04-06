exec('eqm/engine_f16.sci');
exec('eqm/eqm.sci');

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
    cos_beta = cos(X(3));
    sin_beta = sin(X(3));
    if(params.coordinated_turn) then
        //logic coordinated_turn
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
