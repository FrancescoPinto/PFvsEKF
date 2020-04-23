%FUNZIONA, MA per far convergere il particle filter devi fare parameter tuning ...
%    ed anche per far funzionare l'EKF
endT = 400;
startT = 1;
time = startT:1:endT;
x = zeros(2,endT+1); % we compute x(t+1) at t= 300 ... 
y = zeros(1,endT);  %we compute y(t) at t = 300
%x initial conditions
mu_x0 = [1;1];
x(:,1) = [0.7;0.7]; %the robot is always at relative position (4,8) at initialization
F = eye(2);
G = -eye(2);


base_sub = 'Saved_Data';
specific_sub = 'Mixture_Models';
experiment = 'optimal_values';


%% NON GAUSSIAN EXTENSION (no moving target)
%v1 GMM
p_v1 = [0.5 0.5];
mu_gm_v1 = [-0.25 0.25;0.25 -0.25]; % 2 gaussians, the average of the two averages is the true pos
sigma_gm_v1 = [0.01 0; 0 0.01]; % shared diagonal covariance matrix
gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);
%when you need to know the pdf: pdf(gm,[x y])
gmPDF = @(x,y)reshape(pdf(gm_v1,[x(:) y(:)]),size(x)); 
%f = fsurf(gmPDF,[-10 10]);
%title('Probability Densitexperiment,y Function of GMM');
%saveas(f,fullfile(base_sub,specific_sub,experiment,'pdfv1_fsurf_GMM.png'))
%waitfor(f)
%f = fcontour(gmPDF,[-10 10]);
%title('Contour lines of pdf');
%rng('default'); % For reproducibility
v1 = random(gm_v1,endT);
%hold on
%scatter(v1(:,1),v1(:,2),10,'.') % Scatter plot with points of size 10
%title('Contour lines of pdf and Simulated Data');
%saveas(f,fullfile(base_sub,specific_sub,experiment,'pdfv1_contour_GMM.png'))
%waitfor(f)

%v2 GMM
p_v2 = [0.5 0.5];
mu_gm_v2 = [-1;1]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
sigma_gm_v2 = 0.5; % 0.1,0.5shared sigma 
gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);
P = pdf(gm_v2,[-10:0.01:10].');
%f = figure();
%plot([-10:0.01:10].',P);
%title('1D Gaussian mixture model and samples');
%rng('default'); % For reproducibility
v2 = random(gm_v2,endT);
%hold on
%plot(v2,zeros(length(v2),1),'x')
%saveas(f,fullfile(base_sub,specific_sub,experiment,'pdfv2_plot_GMM.png'))
%waitfor(f)


%Approximate the GMM with a Gaussian
[muHat_v2,sigmaHat_v2] = normfit(v2)
[muHat_v1,sigmaHat_v1] = normfit(v1)
P1 = diag(sigmaHat_v1); %P1 = var[x(1)] we arbitrarily choose to initialize P1  with V1
mu_v2 = ceil(muHat_v2);
V1 = diag(sigmaHat_v1);
V2 = sigmaHat_v2;
%{
%Formulae for avg and cov of GMM:
%https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
approx_mu_gm_v1 = mu_gm_v1(1,:)*p_v1(1) + mu_gm_v1(2,:)*p_v1(2)
%approx_sigma_gm_v1 = p_v1(1)*sigma_gm_v1(1,1) + p_v1(2)*sigma_gm_v1(2,2) +p_v1(1)*p_v1(2)*...
    ((mu_gm_v1(1,:)-mu_gm_v1(2,:)))'*((mu_gm_v1(1,:)-mu_gm_v1(2,:)))
approx_sigma_gm_v1 = - 

approx_mu_gm_v2 = mu_gm_v2(1,:)*p_v2(1) + mu_gm_v2(2,:)*p_v2(2)
approx_sigma_gm_v2 = p_v2(1)*sigma_gm_v2(1,1) + p_v2(2)*sigma_gm_v2(1,1) +p_v2(1)*p_v2(2)*...
    ((mu_gm_v2(1,:)-mu_gm_v2(2,:)))'*((mu_gm_v2(1,:)-mu_gm_v2(2,:)))
% DEVI MODIFICARE ... come sotto 
%da fare: P1, V1, mu_v2, V2
%ATTENTO: devi modificare alcune cose ALMENO nel particle filter!!!!
P1 = approx_sigma_gm_v1; %P1 = var[x(1)] we arbitrarily choose to initialize P1  with V1
mu_v2 = approx_mu_gm_v2;
V1 = approx_sigma_gm_v1;
V2 = approx_sigma_gm_v2;
%}


%%%%%%%%%%%%%%%%%%%%%%%%
%%OPTIMAL VALUES
%Mixture_Models
    %constant_vel
        %EKF
            %optim MSE: 11.53
            %V1 = 0.65, V2 = 0.15
        %PF
            %optim PF: 13.12
            %v1 = 0.15  V2 = 0.05
    %circular
        %EKF
            %optim MSE: 104,4042
            %v1 = 0.7, V2 = 0
        %PF
            %optim MSE: 104.4652
            %v1 = 0.15, V2 = 0.45
    %random
        %EKF
            %optim MSE: 5.7576
            %v1 = 0.75, V2 = 0.05
        %PF
            %optim MSE: 6.8899
            %v1 = 0.65, V2 = 0


%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v1'),v1)
%save_matrix_to_file(fullfile(base_sub,specific_sub,'noise_v2'),v2)
v1 = readmtx(fullfile(base_sub,specific_sub,'noise_v1'),size(v1,1),size(v1,2),'float64');
v2 = readmtx(fullfile(base_sub,specific_sub,'noise_v2'),size(v2,1),size(v2,2),'float64');
v1 = 0.2*v1;
v2 = 0.2*v2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Constant velocity robot test
%prima 0.05
u = repmat([0.01;0.01],1,endT+1);
numparticles = 500;
test_name = 'Constant_Velocity_Test';
%estimated (via MSE minimization)
p_v1 = [0.5 0.5];
mu_gm_v1 = [-0 0;0 -0]; % 2 gaussians, the average of the two averages is the true pos
sigma_gm_v1 = [0.001 0; 0 0.001]; % opt 0.2 shared diagonal covariance matrix
gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);

p_v2 = [0.5 0.5];
mu_gm_v2 = [-0;0]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
sigma_gm_v2 = 0.5; % 0.1,0.5shared sigma 
gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);

EKF_V1 = [0.0500 0; 0 0.0500];
EKF_V2 = 0.1;
EKF_P1 = EKF_V1;
PF_V1 = []; %not used
PF_V2 = [];
PF_P1 = [];

FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',true,base_sub,specific_sub,experiment,gm_v1,gm_v2)
%EKF: 0.41
%PF: 0.39<- wins


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
temp_time = [time endT+1];
%%TEST ROBOT CIRCULAR TRAJECTORY 
u = [0.1*sin(temp_time/(endT/4));0.1*cos(temp_time/(endT/4))];
numparticles = 500;
test_name = 'Circular_Test';
%estimated (via MSE minimization)
p_v1 = [0.5 0.5];
mu_gm_v1 = [-0 0;0 -0]; % 2 gaussians, the average of the two averages is the true pos
sigma_gm_v1 = [0.001 0; 0 0.001]; % shared diagonal covariance matrix
gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);

p_v2 = [0.5 0.5];
mu_gm_v2 = [0;0]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
sigma_gm_v2 = 0.5; % 0.1,0.5shared sigma 
gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);

EKF_V1 = [0.05 0; 0 0.05];
EKF_V2 = 0;
EKF_P1 = EKF_V1;
PF_V1 = []; %not used
PF_V2 = [];
PF_P1 = [];

FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',true,base_sub,specific_sub,experiment,gm_v1,gm_v2)
%EKF 26
%PF 1 <- wins
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%RANDOM MOVEMENT ROBOT
%{
u = randn(2,endT+1)*0.4;  % we give input u(t) at t = 300
numparticles = 500;
test_name = 'Random_Test';

%estimated (via MSE minimization)
p_v1 = [0.5 0.5];
mu_gm_v1 = [-0 0; 0 -0]; % 2 gaussians, the average of the two averages is the true pos
sigma_gm_v1 = [0.001 0; 0  0.001]; % shared diagonal covariance matrix
gm_v1 = gmdistribution(mu_gm_v1,sigma_gm_v1,p_v1);

p_v2 = [0.5 0.5];
mu_gm_v2 = [0;0]; % 2 gaussians, mu_0 = [-4], mu_1 = [7]
sigma_gm_v2 = 0.5; % 0.1,0.5shared sigma 
gm_v2 = gmdistribution(mu_gm_v2,sigma_gm_v2,p_v2);

EKF_V1 = [ 1.05 0; 0 1.05];
EKF_V2 = 0.25;
EKF_P1 = EKF_V1;
PF_V1 = []; %not used
PF_V2 = [];
PF_P1 = [];

FUNCTION_full_test(test_name,x,u,F,G,v1.',v2.',mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,'0gaussian_mixture',true,base_sub,specific_sub,experiment,gm_v1,gm_v2)
%EKF 82
%PF 2
%}
