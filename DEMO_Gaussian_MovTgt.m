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


base_sub = 'Saved_Data';
specific_sub = 'Gaussian_MovTgt';
experiment = 'optimal_values';



true_V1 = [0.01 0 0 0;
      0 0.01 0 0;
      0 0 0 0;
      0 0 0 0];
true_V2 = 0.01;%V2 = 0.08;
mu_v1 = 0;
mu_v2 = 0;
%random noise
v1 = true_V1*randn(4,endT) + mu_v1; 
v2 = true_V2*randn(1,endT) + mu_v2;

%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v1'),v1)
%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v2'),v2)
v1 = readmtx(fullfile(base_sub,specific_sub,'noise_v1'),size(v1,1),size(v1,2),'float64');
v2 = readmtx(fullfile(base_sub,specific_sub,'noise_v2'),size(v2,1),size(v2,2),'float64');

v1=v1*0.2;
v2=v2*0.2;
%x initial conditions
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
   
%%TEST CONSTANT VELOCITY ROBOT

EKF_V1 = [0.2 0 0 0; %0.9 %l'ottimo dovrebbe essere 1.25 .. ma è puro caso 
      0 0.2 0 0;
      0 0 0 0;
      0 0 0 0]; %0.1
EKF_V2 = 0.01; %0.6
EKF_P1 = EKF_V1; 
PF_V1 = [0.001 0 0 0; %0.1
      0 0.001 0 0;
      0 0 0 0;
      0 0 0 0]; %0.1
PF_V2 = 0.45; %0.45
PF_P1 = PF_V1; 


u = repmat([0.004;0.004],1,endT+1);
numparticles = 1500;
test_name = 'Constant_Velocity_Test';
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',true,base_sub,specific_sub,experiment)
%1.390
%1.39 PAREGGIO

temp_time = [time endT+1];
%%TEST ROBOT CIRCULAR TRAJECTORY 
u = [sin(temp_time/(endT/4))*0.04;cos(temp_time/(endT/4))*0.04];
%u = [40*sin(time); 40*cos(time)];
numparticles = 5000; %passare da 1000 a 10000 migliora molto la situazione
EKF_V1 = [0.85 0 0 0;
      0 0.85 0 0;
      0 0 0 0;
      0 0 0 0]; %0.15
EKF_V2 = 1.95; %1.15
EKF_P1 = EKF_V1; 
PF_V1 = [0.001 0 0 0;
      0 0.001 0 0;
      0 0 0 0;
      0 0 0 0]; %0.2
PF_V2 = 0.5; %1
PF_P1 = PF_V1; 
test_name = 'Circular_Test';
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',true,base_sub,specific_sub,experiment)
%1.67
%1.64

%%RANDOM MOVEMENT ROBOT
%u = randn(2,endT)*40;  % we give input u(t) at t = 300
u = randn(2,endT+1)*0.01;  % we give input u(t) at t = 300
numparticles = 1000;
EKF_V1 = [0.1 0 0 0; %l'ottimo dovrebbe essere 1.3, ma è puro caso
      0 0.1 0 0;
      0 0 0 0;
      0 0 0 0]; %0.1
EKF_V2 = 1.5;
EKF_P1 = EKF_V1; 
PF_V1 = [0.001 0 0 0; %0.1
      0 0.001 0 0;
      0 0 0 0;
      0 0 0 0];
PF_V2 = 0.5; %0.5
PF_P1 = PF_V1; 
test_name = 'Random_Test';
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'1gaussian',true,base_sub,specific_sub,experiment)
%1.52
%1.59


