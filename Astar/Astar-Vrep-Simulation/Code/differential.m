function out = differential(ul,ur,x_prev,y_prev,theta_prev,res)
    time = 1;
    scaling = 2;
    %% robot dimensions in m
    r = 0.035;
    L = 0.23;
    
    %% Converting rpm to radian per second
    ul = (ul/60)*2*pi;
    ur = (ur/60)*2*pi;
    ul=ul/scaling;
    ur=ur/scaling;
    theta_dot =((r/L)*(ur-ul));
    theta = theta_dot*time + theta_prev;
    x_dot = ((r/2)*(ur+ul))*cos(theta);
    y_dot = ((r/2)*(ur+ul))*sin(theta);
    x = x_dot*time + x_prev;
    y = y_dot*time + y_prev;

%     x = floor(x/res)*res;
%     y = floor(y/res)*res;
   
    out = [x,y,theta,x_dot,y_dot,theta_dot];
end