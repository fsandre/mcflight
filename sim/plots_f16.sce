subplot(211)
plot(t, elev_step);
xgrid
xlabel('Time(s)');
ylabel('Elevator(deg)')

subplot(212)
plot(t,nzs_g);
xgrid
xlabel('Time(s)');
ylabel('Nz(g)');
