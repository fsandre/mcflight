//<-- NO CHECK REF -->
exec('eqm/engine_f16.sci');
ref_csv = read_csv('tests/engine_f16_sample.csv');
nlin = size(ref_csv)(1);
for i = 2:nlin
    pow = strtod(ref_csv(i,1));
    alt_ft = strtod(ref_csv(i,2));
    mach = strtod(ref_csv(i,3));
    thrust_expected = strtod(ref_csv(i,4));
    [thrust_result] = thrust(pow, alt_ft, mach);
    assert_checktrue(abs(thrust_expected-thrust_result)<=1);
end
disp('All calculations in (thrust_pound) <= (1).');
