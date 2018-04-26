exec('eqm/aerodata_f16.sci');
exec('eqm/engine_f16.sci');

function params = load_f16()
    mass = struct('AXX',9496.0, 'AYY', 55814.0, 'AZZ', 63100.0, 'AXZ', 982.0);
    mass.AXZ2 = mass.AXZ**2;
    mass.XPQ = mass.AXZ*(mass.AXX-mass.AYY+mass.AZZ);
    mass.GAM = mass.AXX*mass.AZZ-mass.AXZ**2;
    mass.XQR = mass.AZZ*(mass.AZZ-mass.AYY)+mass.AXZ2;
    mass.ZPQ = (mass.AXX-mass.AYY)*mass.AXX+mass.AXZ2;
    mass.YPR = mass.AZZ - mass.AXX;
    mass.weight_pound = 20490.446;
    g0_ftps2 = 32.17;
    mass.mass_slug = mass.weight_pound/g0_ftps2;
    
    geom = struct('wing_ft2', 300, 'wingspan_ft', 30, 'chord_ft', 11.32, 'xcgr_mac', 0.35);
    geom.engmomenthx_slugft2ps = 160;
    params.mass = mass;
    params.geom = geom;
endfunction
