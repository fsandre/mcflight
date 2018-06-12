//<-- NO CHECK REF -->
exec('models/engine_data.sci');
ref_csv = read_csv('tests/engine_f16_sample.csv');
nlin = size(ref_csv)(1);
for i = 2:nlin
    throttle_u = strtod(ref_csv(i,1));
    initial.throttle_pow = strtod(ref_csv(i,2));
    alt_ft = strtod(ref_csv(i,3));
    mach = strtod(ref_csv(i,4));
    thrust_expected = strtod(ref_csv(i,5));
    importXcosDiagram('tests/models/engine_test.zcos');
    scicos_simulate(scs_m);
    
    assert_checktrue(abs(thrust_expected-thrust_out.values)<=1);
end
disp('All calculations in (thrust_out.values) <= (1).');
