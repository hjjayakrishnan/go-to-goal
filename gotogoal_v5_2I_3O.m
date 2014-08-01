

clear all
close all
clc

% Simulation length (in steps)
k_max = 500; % you may need to make this larger

% Initial positions
x_0 = 0;
y_0 = 0;
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
hold on

% Position references
x_r_c = 1;
y_r_c = 0.5;
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','g')
hold on

% Angular position reference
theta_r_c = atan((y_r_c - y_0)/(x_r_c - x_0)); % is this correct?

% Reference vectors (constant here, but could be time-varying)
x_r = x_r_c*ones(k_max,1);
y_r = y_r_c*ones(k_max,1);
theta_r = theta_r_c*ones(k_max,1);

% Preallocate memory
x = zeros(k_max,1); % position on x axis
y = zeros(k_max,1); % position on y axis
v = zeros(k_max,1); % velocity
theta = zeros(k_max,1); % angular position
omega = zeros(k_max,1); % angular velocity
x_e = zeros(k_max,1); % error in x
y_e = zeros(k_max,1); % error in y
theta_e = zeros(k_max,1); % error in theta

% Time step
t = 0.1;

% Controller (you need to design this properly!)
% (for example, look up feedback linearization)
K = ones(2,3); % I just made this one up

for k = 1:k_max
    
    % Update states
    x(k+1) = x(k) + t*cos(theta(k))*v(k);
    y(k+1) = y(k) + t*sin(theta(k))*v(k);
    theta(k+1) = theta(k) + t*omega(k);
    
    % Calculate errors
    x_e(k) = x_r(k) - x(k);
    y_e(k) = y_r(k) - y(k);
    theta_e(k) = theta_r(k) - theta(k);
    
    % Calculate control inputs
    v(k+1) = K(1,:)*[x_e(k);y_e(k);theta_e(k)];
    omega(k+1) = K(2,:)*[x_e(k);y_e(k);theta_e(k)];
    
    pause(0.025)
    if k > 2
    c = [0.4 0 0] + (k/k_max)*[0.6 0 0];
    plot(x(k),y(k),'o','LineWidth',1,'MarkerSize',8,...
    'MarkerEdgeColor',c,'MarkerFaceColor',c)
    hold on
    grid on
    axis([-abs(x_r_c+0.1) abs(x_r_c+0.1) -abs(y_r_c+0.1) abs(y_r_c+0.1)])
    title(sprintf('iteration = %3i, x = %2.2f, y = %2.2f, theta = %2.2f',k,x(k),y(k),theta(k)))
    end
    
end