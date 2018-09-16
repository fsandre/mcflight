//<-- NO CHECK REF -->
exec('trim/trim_f16.sci');
// Tests based on values in Table 3.6-2 of Steven And Lewis 2nd edition
trim_csv = read_csv('tests/trim_straight_level.csv');
nlin = size(trim_csv)(1);
mprintf('alpha_deg,beta_deg,elev_deg,ail_deg,rudder_deg,VT_ftps,p_rps,q_rps,r_rps,xcg_mac,CX,CY,CZ,Cl,Cm,Cn\n');
for i = 2:nlin
    v_ref = strtod(trim_csv(i,1));
    throttle_ref = strtod(trim_csv(i,2));
    aoa_ref = strtod(trim_csv(i,3));
    elev_ref = strtod(trim_csv(i,4));
    [X, controls, params] = trim_straight_level(v_ref);
    [XD, outputs] = eqm(0, X, controls, params);
    mprintf('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,',X(2)/%pi*180, X(3)/%pi*180, controls.elev_deg, controls.ail_deg, controls.rudder_deg, X(1));
    mprintf('%.3f,%.3f,%.3f,%.3f,',X(7), X(8), X(9), params.xcg);
    mprintf('%.3f,%.3f,%.3f,',outputs.aero_forces(1),outputs.aero_forces(2),outputs.aero_forces(3));
    mprintf('%.3f,%.3f,%.3f\n',outputs.aero_moments(1),outputs.aero_moments(2),outputs.aero_moments(3));
    
    controls.ail_deg = +10;
    [XD, outputs] = eqm(0, X, controls, params);
    mprintf('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,',X(2)/%pi*180, X(3)/%pi*180, controls.elev_deg, controls.ail_deg, controls.rudder_deg, X(1));
    mprintf('%.3f,%.3f,%.3f,%.3f,',X(7), X(8), X(9), params.xcg);
    mprintf('%.3f,%.3f,%.3f,',outputs.aero_forces(1),outputs.aero_forces(2),outputs.aero_forces(3));
    mprintf('%.3f,%.3f,%.3f\n',outputs.aero_moments(1),outputs.aero_moments(2),outputs.aero_moments(3));
    
    controls.ail_deg = -10;
    [XD, outputs] = eqm(0, X, controls, params);
    mprintf('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,',X(2)/%pi*180, X(3)/%pi*180, controls.elev_deg, controls.ail_deg, controls.rudder_deg, X(1));
    mprintf('%.3f,%.3f,%.3f,%.3f,',X(7), X(8), X(9), params.xcg);
    mprintf('%.3f,%.3f,%.3f,',outputs.aero_forces(1),outputs.aero_forces(2),outputs.aero_forces(3));
    mprintf('%.3f,%.3f,%.3f\n',outputs.aero_moments(1),outputs.aero_moments(2),outputs.aero_moments(3));
    
    controls.ail_deg = 0;
    controls.rudder_deg = +15;
    [XD, outputs] = eqm(0, X, controls, params);
    mprintf('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,',X(2)/%pi*180, X(3)/%pi*180, controls.elev_deg, controls.ail_deg, controls.rudder_deg, X(1));
    mprintf('%.3f,%.3f,%.3f,%.3f,',X(7), X(8), X(9), params.xcg);
    mprintf('%.3f,%.3f,%.3f,',outputs.aero_forces(1),outputs.aero_forces(2),outputs.aero_forces(3));
    mprintf('%.3f,%.3f,%.3f\n',outputs.aero_moments(1),outputs.aero_moments(2),outputs.aero_moments(3));
    
    controls.ail_deg = 0;
    controls.rudder_deg = -15;
    [XD, outputs] = eqm(0, X, controls, params);
    mprintf('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,',X(2)/%pi*180, X(3)/%pi*180, controls.elev_deg, controls.ail_deg, controls.rudder_deg, X(1));
    mprintf('%.3f,%.3f,%.3f,%.3f,',X(7), X(8), X(9), params.xcg);
    mprintf('%.3f,%.3f,%.3f,',outputs.aero_forces(1),outputs.aero_forces(2),outputs.aero_forces(3));
    mprintf('%.3f,%.3f,%.3f\n',outputs.aero_moments(1),outputs.aero_moments(2),outputs.aero_moments(3));
end
