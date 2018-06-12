//<-- NO CHECK REF -->
exec('eqm/aerodata_f16.sci');
angle_list = -8:4:12;
[S, K, DA, L] = angle_interp(angle_list, -8);
assert_checktrue(abs(S-1)<=1e-7);
assert_checktrue(abs(K-2)<=1e-7);
assert_checktrue(abs(DA+1)<=1e-7);
assert_checktrue(abs(L-1)<=1e-7);
[S, K, DA, L] = angle_interp(angle_list, -9);
assert_checktrue(abs(S-.75)<=1e-7);
assert_checktrue(abs(K-2)<=1e-7);
assert_checktrue(abs(DA+1.25)<=1e-7);
assert_checktrue(abs(L-1)<=1e-7);
[S, K, DA, L] = angle_interp(angle_list, 11);
assert_checktrue(abs(S-5.75)<=1e-7);
assert_checktrue(abs(K-5)<=1e-7);
assert_checktrue(abs(DA-.75)<=1e-7);
assert_checktrue(abs(L-6)<=1e-7);

CX = [
    -.099   -.081   -.081   -.063   -.025   .044    .097    .113    .145    .167    .174    .166
    -.048   -.038   .040    -.021   .016    .083    .127    .137    .162    .177    .179    .167
    -.022   -.020   -.021   -.004   .032    .094    .128    .130    .154    .161    .155    .138
    -.040   -.038   -.039   -.025   .006    .062    .087    .085    .100    .110    .104    .091
    -.083   -.073   -.076   -.072   -.046   .012    .024    .025    .043    .053    .047    .040
    ];
assert_checktrue(abs(coef_alpha_elev(CX,-10,-24) + .099)<=1e-7);
assert_checktrue(abs(coef_alpha_elev(CX,-10,24) + .083)<=1e-7);
assert_checktrue(abs(coef_alpha_elev(CX,45,-24) - .166)<=1e-7);
assert_checktrue(abs(coef_alpha_elev(CX,45,24) - .040)<=1e-7);
assert_checktrue(abs(coef_alpha_elev(CX,11,-12) - 0.0294)<=1e-7);

A = [
    0  1 2 3 -4.1
    4  3 2 1  0
    -1 2 3 3  4 
    ];
x_list = -5:5:5;
y_list = -3:3:9;
assert_checktrue(abs(coef_alpha_beta(A,-3,-5, y_list, x_list) - 0)<=1e-7);
assert_checktrue(abs(coef_alpha_beta(A, 9,-5, y_list, x_list) + 4.1)<=1e-7);
assert_checktrue(abs(coef_alpha_beta(A,-3, 5, y_list, x_list) + 1)<=1e-7);
assert_checktrue(abs(coef_alpha_beta(A, 9, 5, y_list, x_list) - 4)<=1e-7);
assert_checktrue(abs(coef_alpha_beta(A,1.5,0.1, y_list, x_list) - 2.5)<=1e-7);
disp('All tests passed...');
