clear all
close all
clc

% Simulation length (in steps)
k_max = 50; % make this larger if needed

% Initial positions
x_0 = 0;
y_0 = 0;

% Position references
x_r_c = 1;
y_r_c = 4;

% Error threshold
error_threshold = 0.2;

subplot(2,2,1)
hold on
grid on
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
viscircles([x_r_c y_r_c],error_threshold);
axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])

subplot(2,2,2)
hold on
grid on
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
viscircles([x_r_c y_r_c],error_threshold);
axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])

subplot(2,2,3)
hold on
grid on
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
viscircles([x_r_c y_r_c],error_threshold);
axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])

subplot(2,2,4)
hold on
grid on
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
plot(x_r_c,y_r_c,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
viscircles([x_r_c y_r_c],error_threshold);
axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])

% Constant velocity magnitude
v_constant = 1;

% Reference vectors (constant here, but could be time-varying)
x_r = x_r_c*ones(k_max,1);
y_r = y_r_c*ones(k_max,1);

% Preallocate memory, unicycle 1
x1 = zeros(k_max,1); % position on x axi
y1 = zeros(k_max,1); % position on y axis
v1 = zeros(k_max,1); % velocity
theta1 = zeros(k_max,1); % angular position
omega1 = zeros(k_max,1); % angular velocity
x_e1 = zeros(k_max,1); % error in x
y_e1 = zeros(k_max,1); % error in y
theta_e1 = zeros(k_max,1); % error in theta
theta_d1 = zeros(k_max,1); % desired theta vector

% Preallocate memory, unicycle 2
x2 = zeros(k_max,1); % position on x axis
y2 = zeros(k_max,1); % position on y axis
v2 = zeros(k_max,1); % velocity
theta2 = zeros(k_max,1); % angular position
omega2 = zeros(k_max,1); % angular velocity
x_e2 = zeros(k_max,1); % error in x
y_e2 = zeros(k_max,1); % error in y
theta_e2 = zeros(k_max,1); % error in theta
theta_d2 = zeros(k_max,1); % desired theta vector

% Preallocate memory, unicycle 3
x3 = zeros(k_max,1); % position on x axis
y3 = zeros(k_max,1); % position on y axis
v3 = zeros(k_max,1); % velocity
theta3 = zeros(k_max,1); % angular position
omega3 = zeros(k_max,1); % angular velocity
x_e3 = zeros(k_max,1); % error in x
y_e3 = zeros(k_max,1); % error in y
theta_e3 = zeros(k_max,1); % error in theta
theta_d3 = zeros(k_max,1); % desired theta vector

% Preallocate memory, unicycle 4
x4 = zeros(k_max,1); % position on x axis
y4 = zeros(k_max,1); % position on y axis
v4 = zeros(k_max,1); % velocity
theta4 = zeros(k_max,1); % angular position
omega4 = zeros(k_max,1); % angular velocity
x_e4 = zeros(k_max,1); % error in x
y_e4 = zeros(k_max,1); % error in y
theta_e4 = zeros(k_max,1); % error in theta
theta_d4 = zeros(k_max,1); % desired theta vector

% Time step
T = 0.1;

% Controller gain
k1 = 0.5; % unicycle 1
k2 = 1; % unicycle 2
k3 = 5; % unicycle 3
k4 = 10; % unicycle 4

for t = 1:k_max
    
    % Decide whether to stop the unicycle 1 or not
    if norm([x_r_c;y_r_c] - [x1(t);y1(t)],2) < error_threshold
        v1(t) = 0;
    else
        v1(t) = v_constant;
    end
    
    % Decide whether to stop the unicycle 2 or not
    if norm([x_r_c;y_r_c] - [x2(t);y2(t)],2) < error_threshold
        v2(t) = 0;
    else
        v2(t) = v_constant;
    end
    
    % Decide whether to stop the unicycle 3 or not
    if norm([x_r_c;y_r_c] - [x3(t);y3(t)],2) < error_threshold
        v3(t) = 0;
    else
        v3(t) = v_constant;
    end
    
    % Decide whether to stop the unicycle 4 or not
    if norm([x_r_c;y_r_c] - [x4(t);y4(t)],2) < error_threshold
        v4(t) = 0;
    else
        v4(t) = v_constant;
    end
    
    % Update states, unicycle 1
    x1(t+1) = x1(t) + T*cos(theta1(t))*v1(t)
    y1(t+1) = y1(t) + T*sin(theta1(t))*v1(t);
    theta1(t+1) = theta1(t) + T*omega1(t);
    
    % Update states, unicycle 2
    x2(t+1) = x2(t) + T*cos(theta2(t))*v2(t);
    y2(t+1) = y2(t) + T*sin(theta2(t))*v2(t);
    theta2(t+1) = theta2(t) + T*omega2(t);
    
    % Update states, unicycle 3
    x3(t+1) = x3(t) + T*cos(theta3(t))*v3(t);
    y3(t+1) = y3(t) + T*sin(theta3(t))*v3(t);
    theta3(t+1) = theta3(t) + T*omega3(t);
    
    % Update states, unicycle 4
    x4(t+1) = x4(t) + T*cos(theta4(t))*v4(t);
    y4(t+1) = y4(t) + T*sin(theta4(t))*v4(t);
    theta4(t+1) = theta4(t) + T*omega4(t);
    
    % Calculate desired theta, unicycle 1
    theta_d1(t) = atan((y_r(t) - y1(t))/(x_r(t) - x1(t)));
    
    % Calculate desired theta, unicycle 2
    theta_d2(t) = atan((y_r(t) - y2(t))/(x_r(t) - x2(t)));
    
    % Calculate desired theta, unicycle 3
    theta_d3(t) = atan((y_r(t) - y3(t))/(x_r(t) - x3(t)));
    
    % Calculate desired theta, unicycle 4
    theta_d4(t) = atan((y_r(t) - y4(t))/(x_r(t) - x4(t)));
    
    % Calculate error in theta, unicycle 1
    theta_e1(t) = theta_d1(t) - theta1(t);
    
    % Calculate error in theta, unicycle 2
    theta_e2(t) = theta_d2(t) - theta2(t);
    
    % Calculate error in theta, unicycle 3
    theta_e3(t) = theta_d3(t) - theta3(t);
    
    % Calculate error in theta, unicycle 4
    theta_e4(t) = theta_d4(t) - theta4(t);
    
    % Calculate control input, unicycle 1
    omega1(t+1) = k1*theta_e1(t);
    
    % Calculate control input, unicycle 2
    omega2(t+1) = k2*theta_e2(t);
    
    % Calculate control input, unicycle 3
    omega3(t+1) = k3*theta_e3(t);
    
    % Calculate control input, unicycle 4
    omega4(t+1) = k4*theta_e4(t);
    
    pause(0.05)
    if t > 2
        
        c = [0.5 0.25 0] + (t/k_max)*[0.5 0.25 0];
        
        subplot(2,2,1)
        hold on
        grid on
        plot(x1(t),y1(t),'o','LineWidth',1,'MarkerSize',5,...
        'MarkerEdgeColor',c,'MarkerFaceColor',c)
        axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])
        title(sprintf('k1 = %3.2f, x = %2.2f, y = %2.2f, t = %3i',k1,x1(t),y1(t),t))
        xlabel('x1')
        ylabel('y1')
        
        subplot(2,2,2)
        hold on
        grid on
        plot(x2(t),y2(t),'o','LineWidth',1,'MarkerSize',5,...
        'MarkerEdgeColor',c,'MarkerFaceColor',c)
        axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])
        title(sprintf('k2 = %3.2f, x = %2.2f, y = %2.2f, t = %3i',k2,x2(t),y2(t),t))
        xlabel('x2')
        ylabel('y2')
        
        subplot(2,2,3)
        hold on
        grid on
        plot(x3(t),y3(t),'o','LineWidth',1,'MarkerSize',5,...
        'MarkerEdgeColor',c,'MarkerFaceColor',c)
        axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])
        title(sprintf('k3 = %3.2f, x = %2.2f, y = %2.2f, t = %3i',k3,x3(t),y3(t),t))
        xlabel('x3')
        ylabel('y3')
        
        subplot(2,2,4)
        hold on
        grid on
        plot(x4(t),y4(t),'o','LineWidth',1,'MarkerSize',5,...
        'MarkerEdgeColor',c,'MarkerFaceColor',c)
        axis([-abs(x_r_c+1) abs(x_r_c+1) -abs(y_r_c+1) abs(y_r_c+1)])
        title(sprintf('k4 = %3.2f, x = %2.2f, y = %2.2f, t = %3i',k4,x4(t),y4(t),t))
        xlabel('x4')
        ylabel('y4')
    
    end
    
end