function [out] = PlotSystemBehaviour(x_state, x_state_hat, y, y_corrected, u, t)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
tiledlayout(5,1)
%grid on

% Top plot
ax1 = nexttile;
hold on
plot(ax1,t(2:end), y(:,1), '-', 'LineWidth', 0.5);
plot(ax1,t, x_state(:,1), 'LineWidth', 1);
plot(ax1,t, x_state_hat(:,1), '-', 'LineWidth', 1);
legend('y: measurment','x: internal state', 'x hat: predicted state');
title(ax1,'p - roll angle')
ylabel(ax1,'p [rad/s]')
xlabel(ax1, 'time [s]')
grid on

ax2 = nexttile;
hold on
plot(ax2, t(2:end), rad2deg(y(:,4)), '-', 'LineWidth', 0.5);
plot(ax2, t, rad2deg(x_state(:,4)), 'LineWidth', 1);
plot(ax2, t, rad2deg(x_state_hat(:,4)), '-', 'LineWidth', 1);
legend('y: measurment','x: internal state', 'x hat: predicted state');
title(ax2,'roll angle');
ylabel(ax2,'phi [deg]');
xlabel(ax2, 'time [s]');
grid on

ax3 = nexttile;
hold on
plot(ax3, t(2:end), y(:,2), '--', 'LineWidth', 0.5);
plot(ax3,t, x_state(:,2), '-', 'LineWidth', 1);
plot(ax3,t, x_state_hat(:,2), '-', 'LineWidth', 1);
legend('y: measurment','x: internal state', 'x hat: predicted state');
title(ax3,'r - yaw rate');
ylabel(ax3,'r [rad/s]');
xlabel(ax3, 'time [s]');
grid on

ax4 = nexttile;
hold on
plot(ax4, t(2:end), rad2deg(y_corrected(:,3)), '--', 'LineWidth', 0.5);
plot(ax4, t(2:end), rad2deg(y(:,3)), '--', 'LineWidth', 0.5);
plot(ax4,t, rad2deg(x_state(:,3)), 'LineWidth', 1);
plot(ax4,t, rad2deg(x_state_hat(:,3)), '-', 'LineWidth', 1);
legend('y: corrected', 'y: measurment','x: internal state', 'x hat: predicted state');
title(ax4,'yaw angle (heading)');
ylabel(ax4,'psi [deg]');
xlabel(ax4, 'time [s]');
grid on


ax5 = nexttile;
hold on
%plot(ax5, t, y(:,5), '--', 'LineWidth', 1);
plot(ax5,t, x_state(:,5));
plot(ax5,t, x_state_hat(:,5), '-', 'LineWidth', 1);
legend('x: internal state', 'x hat: predicted state');
title(ax5,'beta');
ylabel(ax5, 'beta [rad]');
xlabel(ax5, 'time [s]');
grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(t(2:end), rad2deg(u(:,1)), 'LineWidth', 1);
hold on
plot(t(2:end), rad2deg(u(:,2)), 'LineWidth', 1);
legend('ailerons','rudder');
title('Actuators state');
ylabel('Deflection [deg]');
xlabel('Time [s]');
grid on

out = 1;
end

