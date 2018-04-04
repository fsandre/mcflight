//<-- NO CHECK REF -->
exec('eqm/eqm.sci');
params.xcg = .4;
controls.throttle = 0.9;
controls.elev_deg = 20;
controls.ail_deg = -15;
controls.rudder_deg = -20;
X= [
    500
    0.5
    -0.2
    -1
    1
    -1
    0.7
    -0.8
    0.9
    1000
    900
    10000
    90
];
DX = eqm(0, X, controls, params, outputs);
DX_expected = [
    -75.23724
    -0.8813491
    -0.4759990
    2.505734
    0.3250820
    2.145926
    12.62679
    0.9649671
    0.5809759
    342.4439
    -266.7707
    248.1241
    -58.68999
]';
error_perc = abs((DX-DX_expected')./DX)*100;
assert_checktrue(error_perc<=0.5);
disp('All calculations with error less than 0.5%');
disp(error_perc);
