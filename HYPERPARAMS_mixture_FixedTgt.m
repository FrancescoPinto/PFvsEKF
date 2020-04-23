%This script makes no use of GUI, it is meant to compute the MSEs when
%varying the parameters (V1,V2,mu,u,numparticles)
endT = 400;
startT = 1;
time = startT:1:endT;
x = zeros(2,endT+1); % we compute x(t+1) at t= 300 ... 
y = zeros(1,endT);  %we compute y(t) at t = 300
%x initial conditions
mu_x0 = [3;7];
x(:,1) = [4;8]; %the robot is always at relative position (4,8) at initialization
F = eye(2);
G = -eye(2);

%% NON GAUSSIAN EXTENSION (no moving target)
%v1 GMM
p_v1 = [0.5 0.5];
mu_gm_v1 = [-0.25 0.25;0.25 -0.25]; % 2 gaussians, the average of the two averages is the true pos
sigma_gm_v1 = [0.01 0; 0 0.01]; % shared diagonal covariance matrix
gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);
%when you need to know the pdf: pdf(gm,[x y])
v1 = random(gm_v1,endT);

%v2 GMM
p_v2 = [0.5 0.5];
mu_gm_v2 = [-1;1]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
sigma_gm_v2 = 0.5; % 0.1,0.5shared sigma 
gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);
v2 = random(gm_v2,endT);

%Approximate the GMM with a Gaussian
[muHat_v2,sigmaHat_v2] = normfit(v2)
[muHat_v1,sigmaHat_v1] = normfit(v1)
P1 = diag(sigmaHat_v1); %P1 = var[x(1)] we arbitrarily choose to initialize P1  with V1
mu_v2 = ceil(muHat_v2);
V1 = diag(sigmaHat_v1);
V2 = sigmaHat_v2;

%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%
%%%%%%%%%%%
upper_bound = 1.5;
V1_values = 0:0.05:upper_bound;
V2_values = 0:0.05:upper_bound;

[V1_ V2_] = meshgrid(V1_values(:), V2_values(:));



const_vel_MSE_EKF = zeros(size(V1_));
const_vel_MSE_PF = zeros(size(V1_));
circular_MSE_EKF = zeros(size(V1_));
circular_MSE_PF = zeros(size(V1_));
random_MSE_EKF = zeros(size(V1_));
random_MSE_PF = zeros(size(V1_));

%Dovresti anche variare lo scale di u, il numparticles ... vedi come fare ... magari fai due suite
visualize = false;
iteration_counter = 0;
total_iterations = size(V1_,1)*size(V1_,2);
for i = 1:1:size(V1_,1)
    for j = 1:1:size(V1_,2)
        '###############'
        'completion percentage'
        iteration_counter = iteration_counter+1;
        iteration_counter/(total_iterations)
        '###############'
        p_v1 = [0.5 0.5];
        mu_gm_v1 = [-0.25 0.25;0.25 -0.25]; % 2 gaussians, the average of the two averages is the true pos
        sigma_gm_v1 = [V1_(i,j) 0; 0 V1_(i,j)]; % shared diagonal covariance matrix
        gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);

        p_v2 = [0.5 0.5];
        mu_gm_v2 = [-1;1]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
        sigma_gm_v2 = V2_(i,j); % 0.1,0.5shared sigma 
        gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);

        EKF_V1 = sigma_gm_v1;
        EKF_V2 = sigma_gm_v2;
        EKF_P1 = EKF_V1;
        PF_V1 = []; %not used
        PF_V2 = [];
        PF_P1 = [];

        %%TEST CONSTANT VELOCITY ROBOT
        %u = repmat([40;40],1,endT);
        u = repmat([0.05;0.05],1,endT+1);
        numparticles = 1000;
        test_name = 'Constant Velocity Robot Test';
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',visualize,gm_v1,gm_v2)
        const_vel_MSE_EKF(i,j) = temp_MSE(1);
        const_vel_MSE_PF(i,j) = temp_MSE(2);
    
    
        temp_time = [time endT+1];
        %%TEST ROBOT CIRCULAR TRAJECTORY 
        u = [sin(temp_time/(endT/4));cos(temp_time/(endT/4))];
        %u = [40*sin(time); 40*cos(time)];
        numparticles = 1000;
        test_name = 'Circular Robot Test';
        %u = randn(2,endT)*0.1;  % we give input u(t) at t = 300
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',visualize,gm_v1,gm_v2)
        circular_MSE_EKF(i,j) = temp_MSE(1);
        circular_MSE_PF(i,j) = temp_MSE(2);


        %%RANDOM MOVEMENT ROBOT
        %u = randn(2,endT+1)*40;  % we give input u(t) at t = 300
        u = randn(2,endT+1)*0.1;  % we give input u(t) at t = 300
        numparticles = 1000;
        test_name = 'Random Movement';
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',visualize,gm_v1,gm_v2)
        random_MSE_EKF(i,j) = temp_MSE(1);
        random_MSE_PF(i,j) = temp_MSE(2);

    end
end

base_sub = 'Saved_Data';
specific_sub = 'Mixture_Models';

save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_const_vel_MSE_EKF'),const_vel_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_const_vel_MSE_PF'),const_vel_MSE_PF)

save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_circular_MSE_EKF'),circular_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_circular_MSE_PF'),circular_MSE_PF)

save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_random_MSE_EKF'),random_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'fixed_tgt_gmm_random_MSE_PF'),random_MSE_PF)

'finito'
'finito'
'finito'
%{
save_matrix_to_file('fixed_tgt_gss_hyperparameters.txt',parameters.')
save_matrix_to_file('fixed_tgt_gss_const_vel_MSE.txt',const_vel_MSE)
save_matrix_to_file('fixed_tgt_gss_circular_MSE.txt',circular_MSE)
save_matrix_to_file('fixed_tgt_gss_random_MSE.txt',random_MSE)
%}
