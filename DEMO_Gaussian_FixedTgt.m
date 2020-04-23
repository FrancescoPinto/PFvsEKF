%% System definition, gaussian assumption
endT = 4000;
startT = 1;
time = startT:1:endT;
x = zeros(2,endT+1); 
y = zeros(1,endT);  
% v1 ~ WGN(0,V1), V1 >= 0
V1 =  [0.06 0; 0 0.06]; 
%V2 = 0.01; %una V2  = 0.5 cos� ampia rende possibile non inizializzare in modo esatto
%V1 =  [400 0; 0 400];
V2 = 0.04;
%V2 = 0.5;
mu_v1 = 0;
mu_v2 = 0;
%random noise
true_V1 = 0.06; %0.03
true_V2 = 0.04; %0.04
v1 = true_V1*randn(2,endT) + mu_v1; 
v2 = true_V2*randn(1,endT) + mu_v2;

%x initial conditions
P1 = V1; %P1 = var[x(1)] initialize P1  with V1
mu_x0 = [-3;-7];
%mu_x0 = [-1;2];
x(:,1) = [4;8]; %the robot is always at relative position (4,8) at initialization
F = eye(2);
G = -eye(2);

base_sub = 'Saved_Data';
specific_sub = 'Gaussian_FixedTgt';
experiment = 'optimal_values';

%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v1'),v1)
%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v2'),v2)
%v1 = readmtx(fullfile(base_sub,specific_sub,'noise_v1'),size(v1,1),size(v1,2),'float64');
%v2 = readmtx(fullfile(base_sub,specific_sub,'noise_v2'),size(v2,1),size(v2,2),'float64');
v1 = v1*0.1;
v2 = v2*0.1;
%OPTIMAL VALUES
%Gaussian_FixedTgt
    %const vel
        %EKF
            %optim MSE: 2.2845
            %V1 = 0.7, V2 = 0.05
        %PF
            %optim MSE: 1.7037
            %V1 = 0.8, V2 = 0.25
    %circular
        %EKF
            %optim MSE: 86.4615
            %V1 = 0.5, V2 = 1
        %PF
            %optim MSE: 82.0078
            %V1 = 0.9, V2 = 0.5
    %random
        %EKF
            %optim MSE: 0.6789
            %V1 = 0.1, V2 = 0.8
        %PF
            %optim MSE: 0.4460
            %V1 = 0.6, V2 = 0.3

%%TEST CONSTANT VELOCITY ROBOT

%u = repmat([40;40],1,endT);
u = repmat([0.05;0.05],1,endT+1);
numparticles = 1000;
EKF_V1 = [0.8 0; 0 0.8]; %qualunque
EKF_V2 = 0.025; 
EKF_P1 = EKF_V1;
PF_V1 = [0.1 0; 0 0.1];
PF_V2 = 0.5;
PF_P1 = PF_V1;
test_name = 'Constant_Velocity_Test';
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian',true, base_sub, specific_sub,experiment)
%MEASURED MSE (reported in report)
%EKF = 1.05 <- wins
%PF = 1.09


temp_time = [time endT+1];
%%TEST ROBOT CIRCULAR TRAJECTORY 
u = [0.3*sin(temp_time/(endT/4));0.3*cos(temp_time/(endT/4))];
%u = [40*sin(time); 40*cos(time)];
numparticles = 1000;
EKF_V1 = [0.02 0; 0 0.02]; %0.02 e 0.3 va bene
EKF_V2 = 0.3;
EKF_P1 = EKF_V1;
PF_V1 = [0.1 0; 0 0.1];
PF_V2 = 0.45;
PF_P1 = PF_V1;
test_name = 'Circular_Test';
%u = randn(2,endT)*0.1;  % we give input u(t) at t = 300
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian',true, base_sub, specific_sub,experiment)
%MEASURED MSE (reported in report)
%EKF = 7.7 <- wins
%PF = 7.56


%%RANDOM MOVEMENT ROBOT
%u = randn(2,endT+1)*40; 
mu_x0 = [3;7];
%mu_x0 = [-1;2];
x(:,1) = [4;8]; %the robot is always at relative position (4,8) at initialization
u = randn(2,endT+1)*0.2;  
numparticles = 500;
EKF_V1 = [0.1 0; 0 0.1]; %relativamente buono 0.001 v1 e 0.005 v2, anche un decimo di entrambi
EKF_V2 = 4; %per ora 0.0000001 e 0.0000005, buono 0.3 e 0.7, meglio 0.1 e 0.8, 0.05 e 0.8, 0.05 e 0.9, 0.05 e 1.2
%CI SIAMO QUASI: 0.05 e 2, meglio .05 e 3, forse meglio .05 e 4, anche 0.04
%e 4.5
%0.04 e 10
EKF_P1 = EKF_V1;
PF_V1 = [0.08 0; 0 0.08]; %buona partenza 0.01 v1 e 0.5 v2, meglio con 0.05 v1, meglio 0.1 v1
PF_V2 = 0.45;%ECCELLENTE: 0.1 v1 e 0.5 v2; meglio 0.1 e 0.4; meglio 0.08 v1 e 0.4
PF_P1 = PF_V1;

test_name = 'Random_Test';
FUNCTION_full_test(test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian',true, base_sub, specific_sub,experiment)
%MEASURED MSE (reported in report)
%EKF = 0.22 <- wins
%PF = 0.15


%OSSERVA: POICHE' VALGONO LE ASSUNZIONI DI GAUSSIANITA' (e probabilmente
%altre assunzion) l'EKF si comporta nettamente meglio del PF
%LA DISCREPANZA TRA IL VALORE 'ottimo' dell'MSE per il PF e quello misurato
%si può supporre sia dovuta alla forte casualità insita nell'uso del PF:
%nel momento in cui si svolgono molti esperimenti, il caso può produrre
%valori molto piccoli di MSE che risultano minimi, ma che lo sono per puro
%caso.
