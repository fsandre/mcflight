exec('sim/sim_f16.sce');
exec('sim/lin_f16.sce');

plot(t, nzs_g, t, y+nzs_g(1));
xgrid
xlabel('Time(s)');
ylabel('Nz(g)');
legend('Non-Linear','Linear');
