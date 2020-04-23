%This script makes no use of GUI, it is meant to compute the MSEs when
%varying the parameters (V1,V2,mu,u,numparticles)
%% System definition; gaussian assumption
endT = 400;
startT = 1;
time = startT:1:endT;
x = zeros(2,endT+1);  
y = zeros(1,endT); 

V1_values = [ 0.01; 0.05; 0.1; 0.5; 2; 2.5; 3; 400];
V2_values = [ 0.01; 0.05; 0.1; 0.5; 2; 2.5; 3; 400];
mu_x0_values = [4 3 2 1 0 -1 -2 -3; 8 7 5 4 3 2 1 0];
mu_indexes = [1;2;3;4;5;6;7;8];

x(:,1) = [4;8]; %the robot is always at relative position (4;8) at initialization
F = eye(2);
G = -eye(2);
mu_v1 = 0;
mu_v2 = 0;

[V1_ V2_ Mu_] = meshgrid(V1_values(:), V2_values(:), mu_indexes(:));
parameters = [V1_(:) V2_(:) Mu_(:)];

const_vel_MSE = [];
circular_MSE = [];
random_MSE = [];
%Dovresti anche variare lo scale di u, il numparticles ... vedi come fare ... magari fai due suite
visualize = false;
for i = 1:1:size(parameters,1)
    V1 = [parameters(i,1) 0; 0 parameters(i,1)];
    V2 = parameters(i,2);
    %random noise
    v1 = V1*randn(2,endT) + mu_v1; 
    v2 = V2*randn(1,endT) + mu_v2;
    P1 = V1;
    mu_x0 = mu_x0_values(:,parameters(i,3));
    
    %%TEST CONSTANT VELOCITY ROBOT
    %u = repmat([40;40],1,endT);
    u = repmat([0.05;0.05],1,endT+1);
    numparticles = 1000;
    test_name = 'Constant Velocity Robot Test';
    const_vel_MSE = [const_vel_MSE FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,P1,V1,mu_v2,V2,time,endT,numparticles,'0gaussian',visualize)];
    

    temp_time = [time endT+1];
    %%TEST ROBOT CIRCULAR TRAJECTORY 
    u = [sin(temp_time/(endT/4));cos(temp_time/(endT/4))];
    %u = [40*sin(time); 40*cos(time)];
    numparticles = 1000;
    test_name = 'Circular Robot Test';
    %u = randn(2,endT)*0.1;  % we give input u(t) at t = 300
    
    circular_MSE = [circular_MSE FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,P1,V1,mu_v2,V2,time,endT,numparticles,'0gaussian',visualize)];
    

    %%RANDOM MOVEMENT ROBOT
    %u = randn(2,endT+1)*40;  % we give input u(t) at t = 300
    u = randn(2,endT+1)*0.1;  % we give input u(t) at t = 300
    numparticles = 1000;
    test_name = 'Random Movement';
    random_MSE = [random_MSE FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,P1,V1,mu_v2,V2,time,endT,numparticles,'0gaussian',visualize)];
end

save_matrix_to_file('fixed_tgt_gss_hyperparameters.txt',parameters.')
save_matrix_to_file('fixed_tgt_gss_const_vel_MSE.txt',const_vel_MSE)
save_matrix_to_file('fixed_tgt_gss_circular_MSE.txt',circular_MSE)
save_matrix_to_file('fixed_tgt_gss_random_MSE.txt',random_MSE)
