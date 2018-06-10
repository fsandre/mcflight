//<-- NO CHECK REF -->
exec('models/aerodynamics_data.sci');
ref_csv = read_csv('tests/aero_coefs.csv');
nlin = size(ref_csv)(1);
for i = 2:nlin
    alpha_deg = strtod(ref_csv(i,1));
    beta_deg = strtod(ref_csv(i,2));
    elev_deg = strtod(ref_csv(i,3));
    ail_deg = strtod(ref_csv(i,4));
    rudder_deg = strtod(ref_csv(i,5));
    VT_ftps = strtod(ref_csv(i,6));
    p_rps = strtod(ref_csv(i,7));
    q_rps = strtod(ref_csv(i,8));
    r_rps = strtod(ref_csv(i,9));
    xcg_mac = strtod(ref_csv(i,10));
    CX_exp = strtod(ref_csv(i,11));
    CY_exp = strtod(ref_csv(i,12));
    CZ_exp = strtod(ref_csv(i,13));
    CL_exp = strtod(ref_csv(i,14));
    CM_exp = strtod(ref_csv(i,15));
    CN_exp = strtod(ref_csv(i,16));
    importXcosDiagram('tests/models/aerodynamics_test.zcos');
    scicos_simulate(scs_m);
    assert_checktrue(abs(CX_exp-CX.values)<=1e-3);
    assert_checktrue(abs(CY_exp-CY.values)<=1e-3);
    assert_checktrue(abs(CZ_exp-CZ.values)<=1e-3);
    assert_checktrue(abs(CL_exp-CL.values)<=1e-3);
    assert_checktrue(abs(CM_exp-CM.values)<=1e-3);
    assert_checktrue(abs(CN_exp-CN.values)<=1e-3);
    mprintf('alpha_deg: %.3f\n',alpha_deg);
    mprintf('>%.3f,%.3f,%.3f,',CX_exp,CY_exp,CZ_exp);
    mprintf('%.3f,%.3f,%.3f\n',CL_exp,CM_exp,CN_exp);
    mprintf('<%.3f,%.3f,%.3f,',CX.values,CY.values,CZ.values);
    mprintf('%.3f,%.3f,%.3f\n',CL.values,CM.values,CN.values);
end
disp('All aerodynamics forces and coefficients <= (1e-3).');
