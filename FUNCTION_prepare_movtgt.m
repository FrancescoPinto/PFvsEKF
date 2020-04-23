function [determ_x, determ_x_global, determ_y, stoch_x,stoch_x_global, stoch_y,mv_tgt_y,target_trajectory] = FUNCTION_prepare_movtgt(x,u,F,G,v1,v2,time,endT,showdet,showstoch,showplots,constant_vel,visualize,base_sub,specific_sub,experiment,test_name)
%FUNCTION_PREPARE_MOVTGT Prepare x and y and trajectory for moving target
%  

for i=1:1:endT
    x(:,i+1) = F*x(:,i)+G*u(:,i);
    y(i) = sqrt(x(1,i)^2 + x(2,i)^2);
end

determ_x =x;
determ_y = y;

[global_robot_pos, determ_x_global] = from_local_to_global(u,determ_x(1:2,:));
target_trajectory = determ_x_global; %if system is deterministic, target trajectory = determ x global

if showdet == true && visualize == true
    visualize_plane(global_robot_pos(1,:),global_robot_pos(2,:),determ_x_global(1,:),determ_x_global(2,:),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'True_Target_Trajectory'],'_'),base_sub,specific_sub,experiment);
end

%stochastic system
for i=1:1:endT
    x(:,i+1) = F*x(:,i)+G*u(:,i)+v1(:,i);
    y(i) = sqrt(x(1,i)^2 + x(2,i)^2) + v2(:,i);
end

stoch_x =x;
stoch_y = y;
[global_robot_pos, stoch_x_global] = from_local_to_global(u,stoch_x(1:2,:));

if showstoch == true && visualize == true
    visualize_plane(global_robot_pos(1,:),global_robot_pos(2,:),stoch_x_global(1,:),stoch_x_global(2,:),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'True_Stoch_Target_Trajectory'],'_'),base_sub,specific_sub,experiment);
end

if showplots == true && visualize == true
    %plot data
    x1_cell = {determ_x(1,1:end-1),stoch_x(1,1:end-1)};
    x2_cell = {determ_x(2,1:end-1),stoch_x(2,1:end-1)};
    x3_cell = {determ_y(1,1:end),stoch_y(1,1:end)};
    plot_multiple(3,join([test_name,'Local_True_System'],'_'), time,x1_cell,x2_cell,x3_cell,{'det','stoch'},base_sub,specific_sub,experiment)

    x1_cell = {determ_x_global(1,1:end-1),stoch_x_global(1,1:end-1)};
    x2_cell = {determ_x_global(2,1:end-1),stoch_x_global(2,1:end-1)};
    plot_multiple(2, join([test_name,'Global_True_System'],'_'), time,x1_cell,x2_cell,{'det','stoch'},base_sub,specific_sub,experiment)
end 

global_robot_pos = robot_from_local_to_global(u);

target_trajectory = zeros(2,endT+1);
target_local_coordinates_trajectory = zeros(2,endT+1);

target_trajectory(:,1) = x(1:2,1);
target_local_coordinates_trajectory(:,1) = x(1:2,1); %initially local = global

for i = 2:1:endT
    target_trajectory(:,i) = target_trajectory(:,i-1)+constant_vel;
    target_local_coordinates_trajectory(:,i) = target_trajectory(:,i) - global_robot_pos(:,i);
end

target_trajectory = target_trajectory(:,1:end-1);
for i=1:1:endT
    mv_tgt_y(:,i) = sqrt(target_local_coordinates_trajectory(1,i)^2 + target_local_coordinates_trajectory(2,i)^2) + v2(i);
end

end

