exec('atmosphere/atmos_constants.sci');
function [T_K, p_Pa, rho_kgpm3] = atmosphere(h_m, deltaIsa)
    Z_m = atmos.r0_m*h_m/(atmos.r0_m-h_m);
    g = atmos.g0_mps2*(atmos.r0_m./(atmos.r0_m+Z_m)).^2
    if ~exists('deltaIsa','local') then
        deltaIsa = 0;
    end
    if(h_m <= 11000) then
        T_K = atmos.T0_K - 6.5*(Z_m/1000.0) + deltaIsa;
        p_Pa = atmos.p0_Pa*(1 - 0.0065*(Z_m/atmos.T0_K)).^5.255877;
    else
        T_K = atmos.T0_K - 6.5*(11000.0/1000.0) + deltaIsa;
        p11 = atmos.p0_Pa*(1 - 0.0065*(11000/atmos.T0_K)).^5.2561;
        p_Pa = p11.*exp(-g/atmos.R/T_K*(h_m-11000.0))
    end
    rho_kgpm3 = p_Pa/atmos.R/T_K;
endfunction
