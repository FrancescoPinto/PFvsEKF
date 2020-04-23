function [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(varargin)
%FUNCTION_TEST_PF Implementation of the Particle Filter (PF) test suite
%   

numparticles = varargin{1};
mode = varargin{2};
F = varargin{3};
G = varargin{4};
y = varargin{5};
u = varargin{6};
time = varargin{7};
endT = varargin{8};
target_trajectory = varargin{9};
determ_x = varargin{10};
determ_x_global = varargin{11};
stoch_x = varargin{12};
stoch_x_global = varargin{13};

if strcmp(mode,'gaussian')
    V1 = varargin{14};
    mu_x0 = varargin{15};
    P1 = varargin{16};
    mu_v2 = varargin{17};
    V2 = varargin{18};
    visualize = varargin{19};
    if visualize == true
        base_sub = varargin{20};
        specific_sub = varargin{21};
        experiment = varargin{22};
        test_name = varargin{23};
    end
    
    part_filt_x = particle_filter( 'gaussian',numparticles, endT, F,G,y,u,V1,mu_x0,P1,mu_v2,V2);
elseif strcmp(mode,'gaussian_mixture')
    gm_v1 = varargin{14};
    mu_x0 = varargin{15};
    P1 = varargin{16};
    gm_v2 = varargin{17};
    visualize = varargin{18};
    if visualize == true
        base_sub = varargin{19};
        specific_sub = varargin{20};
        experiment = varargin{21};    
        test_name = varargin{22};
    end
    part_filt_x = particle_filter( 'gaussian_mixture',numparticles, endT, F,G,y,u,gm_v1,mu_x0,P1,gm_v2);
end

[global_robot_pos, part_filt_x_global] = from_local_to_global(u,part_filt_x(1:2,:));

if visualize == true
    visualize_dots_plane(part_filt_x_global,target_trajectory,join([test_name,'PF_static_dots'],'_'),base_sub,specific_sub,experiment)
    visualize_plane(global_robot_pos(1,1:end-1),global_robot_pos(2,1:end-1),part_filt_x_global(1,1:end-1),part_filt_x_global(2,1:end-1),target_trajectory(1,:),target_trajectory(2,:),join([test_name,'PF_dynamic_dots'],'_'),base_sub,specific_sub,experiment);

    x1_cell = {part_filt_x(1,1:end-1),stoch_x(1,1:endT),determ_x(1,1:endT)};
    x2_cell = {part_filt_x(2,1:end-1),stoch_x(2,1:endT),determ_x(2,1:endT)};
    plot_multiple(2, join([test_name,'Local_PF'],'_'), time,x1_cell,x2_cell,{'pf','stoch','determ'},base_sub,specific_sub,experiment)
    x1_cell = {part_filt_x_global(1,1:end-1),stoch_x_global(1,1:endT),determ_x_global(1,1:endT)};
    x2_cell = {part_filt_x_global(2,1:end-1),stoch_x_global(2,1:endT),determ_x_global(2,1:endT)};
    plot_multiple(2, join([test_name,'Global_PF'],'_'), time,x1_cell,x2_cell,{'pfglob','stochglob','determglob'},base_sub,specific_sub,experiment)
end

end

