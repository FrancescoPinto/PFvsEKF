function [filt_x,filt_x_global] = FUNCTION_test_ekf(F,G,u,P1,V1,y,V2,mu_x0,target_trajectory,time,endT,determ_x,determ_x_global,stoch_x,stoch_x_global,visualize,base_sub,specific_sub,experiment,test_name)
%FUNCTION_TEST_EKF_PF Implementation of the EKF test suite
%   
%EKF
[filt_x,e] = extended_kalman_filter(endT,F,G,u,P1,V1,y,V2,mu_x0);

%visualize on plane
[global_robot_pos, filt_x_global] = from_local_to_global(u,filt_x(1:2,:));
if visualize == true
    visualize_dots_plane(filt_x_global,target_trajectory,join([test_name,'Static_EKF'],'_'),base_sub,specific_sub,experiment)
    size(global_robot_pos)
    size(filt_x_global)
    size(target_trajectory)
    %visualize_plane(global_robot_pos(1,1:end-1),global_robot_pos(2,1:end-1),filt_x_global(1,1:end-1),filt_x_global(2,1:end-1),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'Dynamic_EKF'],'_'),base_sub,specific_sub,experiment);

    x1_cell = {filt_x(1,1:end-1),stoch_x(1,1:endT),determ_x(1,1:endT)};
    x2_cell = {filt_x(2,1:end-1),stoch_x(2,1:endT),determ_x(2,1:endT)};
    plot_multiple(2, join([test_name,'Local_EKF'],'_'), time,x1_cell,x2_cell,{'ekf','stoch','determ'},base_sub,specific_sub,experiment)
    x1_cell = {filt_x_global(1,1:end-1),stoch_x_global(1,1:endT),determ_x_global(1,1:endT)};
    x2_cell = {filt_x_global(2,1:end-1),stoch_x_global(2,1:endT),determ_x_global(2,1:endT)};
    plot_multiple(2, join([test_name,'Global_EKF'],'_'), time,x1_cell,x2_cell,{'ekfglob','stochglob','determ'},base_sub,specific_sub,experiment)
end

end

