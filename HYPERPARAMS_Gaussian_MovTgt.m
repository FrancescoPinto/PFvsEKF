%% TRACKING, still gaussian assumption 
%REFERENCE: http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/pdfs/rob2-12-tracking.pdf
endT = 300;
startT = 1;
time = startT:1:endT;
x = zeros(2,endT+1); % we compute x(t+1) at t= 300 ... 
x(:,1) = [4;8]; %the target is always at relative position (4,8) at initialization
constant_vel = [0.01;0.01];
x = [x;repmat(constant_vel,1,endT+1)];
y = zeros(1,endT);  %we compute y(t) at t = 300
%u = [sin(time).*40; cos(time).*80]
%u = repmat(sin(time).*40,2,1);
%u = repmat([1 3 5 8 5 3 2 -1 -3 -4; -2 1 3 -4 5 6 7 -3 9 -3],1,endT/10);


true_V1 = [0.01 0 0 0;
      0 0.01 0 0;
      0 0 0 0;
      0 0 0 0];
P1 = true_V1;
true_V2 = 0.02;%V2 = 0.08;
mu_v1 = 0;
mu_v2 = 0;
%random noise
v1 = true_V1*randn(4,endT) + mu_v1; 
v2 = true_V2*randn(1,endT) + mu_v2;

upper_bound = 2;
V1_values = 0:0.05:upper_bound;
V2_values = 0:0.05:upper_bound;
[V1_ V2_] = meshgrid(V1_values(:), V2_values(:));


%accurate initializatoin crucial
mu_x0 = [4;8;constant_vel];
T = 1; %time interval, needed to do v*T = displacement
F = [1 0  T  0 ;
     0 1  0  T ;
     0 0  1  0 ;
     0 0  0  1]; %constant velocity model G = -eye(2);

 G = [-1 0 ;
       0 -1;
       0 0 ;
       0 0];

const_vel_MSE_EKF = zeros(size(V1_));
const_vel_MSE_PF = zeros(size(V1_));
circular_MSE_EKF = zeros(size(V1_));
circular_MSE_PF = zeros(size(V1_));
random_MSE_EKF = zeros(size(V1_));
random_MSE_PF = zeros(size(V1_));

   
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
        V1 = [V1_(i,j) 0 0 0 ; 
              0 V1_(i,j) 0 0;
              0 0  1  0 ;
              0 0  0  1];
        V2 = V2_(i,j);
        %random noise
        EKF_P1 = V1; %P1 = var[x(1)] we arbitrarily choose to initialize P1  with V1
        EKF_V1 = V1;
        EKF_V2 = V2;
        PF_V1 = V1;
        PF_V2 = V2;
        PF_P1 = V1;

        u = repmat([0.004;0.004],1,endT+1);
        numparticles = 1500;
        test_name = 'Constant_Velocity_Test';
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',visualize);
        const_vel_MSE_EKF(i,j) = temp_MSE(1);
        const_vel_MSE_PF(i,j) = temp_MSE(2);


        temp_time = [time endT+1];
        %%TEST ROBOT CIRCULAR TRAJECTORY 
        u = [sin(temp_time/(endT/4))*0.04;cos(temp_time/(endT/4))*0.04];
        %u = [40*sin(time); 40*cos(time)];
        numparticles = 5000; %passare da 1000 a 10000 migliora molto la situazione
        test_name = 'Circular_Test';
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',visualize);
        circular_MSE_EKF(i,j) = temp_MSE(1);
        circular_MSE_PF(i,j) = temp_MSE(2);


        %%RANDOM MOVEMENT ROBOT
        %u = randn(2,endT)*40;  % we give input u(t) at t = 300
        u = randn(2,endT+1)*0.01;  % we give input u(t) at t = 300
        numparticles = 1000;
        test_name = 'Random_Test';
        temp_MSE = FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',visualize);
        random_MSE_EKF(i,j) = temp_MSE(1);
        random_MSE_PF(i,j) = temp_MSE(2);

    
    end
end
   

figure
mesh(V1_,V2_,const_vel_MSE_PF)

base_sub = 'Saved_Data';
specific_sub = 'Gaussian_MovTgt';

save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_const_vel_MSE_EKF'),const_vel_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_const_vel_MSE_PF'),const_vel_MSE_PF)

save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_circular_MSE_EKF'),circular_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_circular_MSE_PF'),circular_MSE_PF)

save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_random_MSE_EKF'),random_MSE_EKF)
save_matrix_to_file(fullfile(base_sub,specific_sub,'mov_tgt_gss_random_MSE_PF'),random_MSE_PF)


'finito'
'finito'
'finito'




