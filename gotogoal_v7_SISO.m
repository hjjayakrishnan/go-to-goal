clear all
close all
clc

% Simulation length (in steps)
k_max = 250; % you may need to make this larger

% Initial positions
x_0 = 0;
y_0 = 0;
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
hold on

% Position references
x_r_c = -.9;
y_r_c = 4;
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
hold on

% Reference vectors (constant here, but could be time-varying)
x_r = x_r_c*ones(k_max,1);
y_r = y_r_c*ones(k_max,1);

% Preallocate memory
x = zeros(k_max,1); % position on x axis
y = zeros(k_max,1); % position on y axis
v = zeros(k_max,1); % velocity
theta = zeros(k_max,1); % angular position
omega = zeros(k_max,1); % angular velocity
x_e = zeros(k_max,1); % error in x
y_e = zeros(k_max,1); % error in y
theta_e = zeros(k_max,1); % error in theta
theta_d = zeros(k_max,1); % desired theta
p_e = zeros(k_max,1); % error in position

% Time step
T = 0.1;

% Controller gains
k_v = 0.3; % velocity gain
k_omega = 1; % angular velocity gain

for t = 1:k_max
    
    % Update states
    x(t+1) = x(t) + T*cos(theta(t))*v(t);
    y(t+1) = y(t) + T*sin(theta(t))*v(t);
    theta(t+1) = theta(t) + T*omega(t);
    
    % Calculate error in position
    p_e(t) = norm([x_r_c;y_r_c] - [x(t);y(t)],2);
    
    % Calculate control input 1 (velocity)
    v(t+1) = k_v*p_e(t);
    
    % Calculate desired theta
    switch x_r(t) >= x(t)
        case 1
            theta_d(t) = atan((y_r(t) - y(t))/(x_r(t) - x(t)));
        case 0
            switch y_r(t) >= y(t)
                case 1
                    theta_d(t) = (pi/2) + ...
                        atan(abs(y_r(t) - y(t))/abs(x_r(t) - x(t)));
                case 0
                    theta_d(t) = -(pi/2) - ...
                        atan(abs(y_r(t) - y(t))/abs(x_r(t) - x(t)));
            end
    end
    
    % Calculate error in theta
    theta_e(t) = theta_d(t) - theta(t);
    
    % Calculate control input 2 (angular velocity)
    omega(t+1) = k_omega*theta_e(t);
    
    pause(0.00001)
    if t > 2
        
        c = [0.5 0.25 0] + (t/k_max)*[0.5 0.25 0];
        plot(x(t),y(t),'o','LineWidth',1,'MarkerSize',8,...
        'MarkerEdgeColor',c,'MarkerFaceColor',c)
        hold on
        grid on
       % axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])
        title(sprintf('iteration = %3i, x = %2.2f, y = %2.2f',t,x(t),y(t)))
    
    end
    
end