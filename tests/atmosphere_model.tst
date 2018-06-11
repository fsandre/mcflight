//<-- NO CHECK REF -->
exec('models/atmos_constants.sci');
isa_ref_csv = read_csv('tests/isa_atm_sample.csv');
nlin = size(isa_ref_csv)(1);
for i = 2:nlin
    h_ref = strtod(isa_ref_csv(i,1));
    T_ref = strtod(isa_ref_csv(i,3));
    rho_ref = strtod(isa_ref_csv(i,5));
    p_ref = strtod(isa_ref_csv(i,6));
    importXcosDiagram('tests/models/atmosphere_test.zcos');
    h_in = h_ref;
    scicos_simulate(scs_m);
    assert_checktrue(abs(T_ref-T_out.values)<=1e-3);
    assert_checktrue(abs(p_ref-p_out.values)<=10);
    assert_checktrue(abs(rho_ref-rho_out.values)<=2e-4);
end
disp('All calculations in (temperature_K, pressure_Pa, rho_kgpm3) <= (1e-3, 10, 2e-4).');
