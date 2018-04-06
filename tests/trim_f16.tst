//<-- NO CHECK REF -->
exec('trim/trim_f16.sci');
params.xcg = .35;
params.coordinated_turn = 0;
params.turn_rate_rps = 0.0;
params.roll_rate_rps = 0.0;
params.pitch_rate_rps = 0.0;
params.phi_rad = 0.0;
params.gamma_rad = 0.0;
params.stability_axis_roll = 0;
params.VT_ftps = 140;
params.alt_ft = 0;
function y = costf16(x)
    y = cost_trim_f16(x,params);
endfunction
S0 = [
     .0
     0.0
     0.0
     //0.0
     //0.0
     //0.0
     ];
S = fminsearch(costf16, S0);
disp(S);
