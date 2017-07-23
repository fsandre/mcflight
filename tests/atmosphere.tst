//<-- NO CHECK REF -->
exec('atmosphere/atmosphere.sci');
isa_ref_csv = read_csv('tests/isa_atm_sample.csv');
nlin = size(isa_ref_csv)(1);
for i = 2:nlin
    h_ref = strtod(isa_ref_csv(i,1));
    T_ref = strtod(isa_ref_csv(i,3));
    rho_ref = strtod(isa_ref_csv(i,5));
    p_ref = strtod(isa_ref_csv(i,6));
    [T_K, p_Pa, rho_kgpm3] = atmosphere(h_ref);
    assert_checktrue(abs(T_ref-T_K)<=1e-3);
    assert_checktrue(abs(p_ref-p_Pa)<=10);
    assert_checktrue(abs(rho_ref-rho_kgpm3)<=2e-4);
end
disp('All calculations in (temperature_K, pressure_Pa, rho_kgpm3) <= (1e-3, 10, 2e-4).');
