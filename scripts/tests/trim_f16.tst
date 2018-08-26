//<-- NO CHECK REF -->
exec('trim/trim_f16.sci');
// Tests based on values in Table 3.6-2 of Steven And Lewis 2nd edition
trim_csv = read_csv('tests/trim_straight_level.csv');
nlin = size(trim_csv)(1);
for i = 2:nlin
    v_ref = strtod(trim_csv(i,1));
    throttle_ref = strtod(trim_csv(i,2));
    aoa_ref = strtod(trim_csv(i,3));
    elev_ref = strtod(trim_csv(i,4));
    [X, controls] = trim_straight_level(v_ref);
    mprintf('Speed: %.0f, throttle:(%.3f,%.3f), AOA(deg): (%.3f,%.3f), Elev(deg): (%.3f,%.3f)\n',v_ref,throttle_ref,controls.throttle,aoa_ref,X(2)*180/%pi,elev_ref,controls.elev_deg);
    assert_checktrue(abs(throttle_ref-controls.throttle)<=1e-3);
    assert_checktrue(abs(aoa_ref-X(2)*57.3)<=0.1);
    assert_checktrue(abs(elev_ref-controls.elev_deg)<=0.1);
end
disp('All calculations in (aoa, elev) <= (0.1, 0.1).');
