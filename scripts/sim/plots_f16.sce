subplot(311)
plot(t, elev_step);
xgrid
xlabel('Time(s)');
ylabel('Elevator(deg)')

subplot(312)
plot(t, nzs_g);
xgrid
xlabel('Time(s)');
ylabel('Nz(g)');

subplot(313)
plot(t, y(12,:));
xgrid
xlabel('Time(s)');
ylabel('H(ft)');

