function [determ_x, determ_x_global,determ_y, stoch_x, stoch_x_global, stoch_y,y,target_trajectory] = FUNCTION_prepare_xy(x,u,F,G,v1,v2,time,endT,showdet,showstoch,showplots,visualize,base_sub,specific_sub,experiment, test_name)
%FUNCTION_PREPARE_XY Prepare x, y and trajectory for fixed target
%   
%deterministic system
for i=1:1:endT
    x(:,i+1) = F*x(:,i)+G*u(:,i);
    y(i) = sqrt(x(1,i)^2 + x(2,i)^2);
end

determ_x =x;
determ_y = y;

[global_robot_pos, determ_x_global] = from_local_to_global(u,determ_x);
target_trajectory = repmat(x(:,1),endT);

if showdet == true && visualize == true
    visualize_plane(global_robot_pos(1,:),global_robot_pos(2,:),determ_x_global(1,:),determ_x_global(2,:),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'Plane_True_System_Determ'],'_'),base_sub,specific_sub,experiment);
end

%stochastic system
for i=1:1:endT
    x(:,i+1) = F*x(:,i)+G*u(:,i)+v1(:,i);
    y(i) = sqrt(x(1,i)^2 + x(2,i)^2) + v2(:,i);
end

stoch_x =x;
stoch_y = y;
[global_robot_pos, stoch_x_global] = from_local_to_global(u,stoch_x);

if showstoch == true && visualize == true
    visualize_plane(global_robot_pos(1,2:end),global_robot_pos(2,2:end),stoch_x_global(1,2:end),stoch_x_global(2,2:end),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'Plane_True_System_Stoch'],'_'),base_sub,specific_sub,experiment);
end

if showplots == true && visualize == true
    %plot data
    x1_cell = {determ_x(1,1:end-1),stoch_x(1,1:end-1)};
    x2_cell = {determ_x(2,1:end-1),stoch_x(2,1:end-1)};
    x3_cell = {determ_y(1,1:end),stoch_y(1,1:end)};
    plot_multiple(3, join([test_name,'Local_True_System'],'_'), time,x1_cell,x2_cell,x3_cell,{'det','stoch'},base_sub,specific_sub,experiment)

   
    x1_cell = {determ_x_global(1,1:end-1),stoch_x_global(1,1:end-1)};
    x2_cell = {determ_x_global(2,1:end-1),stoch_x_global(2,1:end-1)};
    plot_multiple(2, join([test_name,'Global_True_System'],'_'), time,x1_cell,x2_cell,{'det','stoch'},base_sub,specific_sub,experiment)
end 

%Simulated sensor 
%l'idea è che il sensore nella realtà non restituisce una misura
%influenzata dall'errore nella predizione di quella che sarà la posizione
%del target, ma semplicemente prende una misura di distanza e basta
%(affetta da errore di misura, ma non influenzata da assunzioni sulla
%posizione del target)
for i=1:1:endT
    y(i) = sqrt(determ_x(1,i)^2 + determ_x(2,i)^2) + v2(:,i); 
end

if showplots == true && visualize == true
    x1_cell = {determ_x(1,1:end-1),stoch_x(1,1:end-1)};
    x2_cell = {determ_x(2,1:end-1),stoch_x(2,1:end-1)};
    x3_cell = {determ_y(1,1:end),y(1,1:end)};
    plot_multiple(3, join([test_name,'Simulated_Sensor_Data'],'_'), time,x1_cell,x2_cell,x3_cell,{'det','stoch'},base_sub,specific_sub,experiment)
end

end

