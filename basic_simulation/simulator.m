clear; close;
p = 0; % roll rate
r = 0; % yaw rate
psi = 0; % yaw angle
phi = 0; % roll angle
beta = 0; % sidesplip angle
delta_a = 0; % aileron actuator deflection <-60deg, 60deg>
delta_r = 0; % rudder actuator deflection



x = [p; r; psi; phi; beta];
u = [delta_a; delta_r];

A = [-11.4540, 2.7185, 0, 0, -19.4399;
    0.5068, -2.9875, 0, 0, 23.3434;
    0, 1, 0, 0, 0;
    1, 0.0926, 0, 0, 0;
    0.0922, -0.9957, 0, 0.3256, -0.4680];

B = [78.4002, -2.7282;
    -3.4690, 13.9685;
    0, 0;
    0, 0;
    0, 0];

C = [1, 0, 0, 0, 0;
    0, 1, 0, 0, 0;
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 0];

D = [0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% continous model
model = ss(A, B, C, D, 'StateName',{'p' 'r', 'psi', 'phi', 'beta'},'InputName',{'delta_a', 'delta_r'});
x0 = [0;0;0;0;0.1];


Tf = 10;
Ts = 0.1;
[uSq,t] = gensig("square",2,Tf,Ts);
uSq = uSq * 2 - 1;
%[uP,~] = gensig("pulse",3,Tf,Ts);
uF = zeros(length(uSq), 1);
u = [uF, uSq*2-1];
y = lsim(model,u,t, x0, 'zoh');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% discerte model
dt = 0.01;                  % simulation step [s]
T = 20;                     % simulation time [s]

model = c2d(model, dt);

[input_r,t] = gensig("sine",1,T,dt);
input_a = zeros(size(t));
input = [input_a, input_a]; % [input_a, input_r];
[y, ~, x_state] = lsim(model, input, t, x0, 'zoh');


%initial(model,x0)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tiledlayout(5,1)
%grid on

% Top plot
ax1 = nexttile;
hold on
plot(ax1,t, y(:,1));
plot(ax1,t, x_state(:,1));
title(ax1,'p')
ylabel(ax1,'p [rad/s]')
grid on

ax2 = nexttile;
hold on
plot(ax2, t, y(:,2));
plot(ax2,t, x_state(:,2));
title(ax2,'r')
ylabel(ax2,'r [rad/s]')
grid on

ax3 = nexttile;
hold on
plot(ax3, t, rad2deg(y(:,3)));
plot(ax3,t, rad2deg(x_state(:,3)));
title(ax3,'yaw angle (heading)')
ylabel(ax3,'psi [deg]')
grid on

ax4 = nexttile;
hold on
plot(ax4, t, rad2deg(y(:,4)));
plot(ax4,t, rad2deg(x_state(:,4)));
title(ax4,'roll angle')
ylabel(ax4,'phi [deg]')
grid on

ax5 = nexttile;
hold on
plot(ax5, t, y(:,5));
plot(ax5,t, x_state(:,5));
title(ax5,'beta')
ylabel(ax5,'??')
grid on
