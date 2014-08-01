clear all
close all
clc
%UAV final destination
r=-1;
s=2;
plot(r,s,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
axis ([-5 5 -5 5]); 
grid on
hold on
%UAV initial position
a=0;
b=0;
plot(a,b,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
hold on
%UAV current co-ordinates
x_old=a;
y_old=b;
x_new=a;
y_new=b;
%gain
k=1.3;
%time step
t=.1;
%error threshold
error_threshold=.1;
% max iteration
Max_iteration=75;

%initial heading
phi_old=0;
phi_new=0;






for m = 1: Max_iteration
    
    % Stop simulation if destination is reached
    if norm([r;s] - [x_new;y_new],2) < error_threshold
        v = 0;
    else
        v = 1;
    end
    
    
  %   phi_desired=vpa(atan((s-y_new)/(r-x_new)));
         switch r >= x_new
        case 1
            phi_desired = vpa(atan((s-y_new)/(r-x_new)));
        case 0
            switch s >= y_new
                case 1
                    phi_desired = (pi/2) + ...
                        vpa(atan(abs((s-y_new)/(r-x_new))));
                case 0
                    phi_desired = -(pi/2) - ...
                        vpa(atan(abs((s-y_new)/(r-x_new))));
            end
        end
      
  
  
        error=phi_desired-phi_old;
        % w: angular velocity
       
        w=k*error
                
        % unicycle model system dynamics
        x_new=x_old + v*cos(phi_old) * t
        y_new=y_old + v*sin(phi_old) * t
        phi_new=t*w+phi_old
        
        pause(t);
                       
          
        plot(x_new,y_new,'*');
        hold on
        x_old=x_new;
        y_old=y_new;
        phi_old=phi_new
             
        
end     
        