upper_bound = 2;
%upper_bound = 1;
upper_bound=1.5;
V1_values = 0:0.05:upper_bound;
V2_values = 0:0.05:upper_bound;
%V1_values = 0:0.02:upper_bound;
%V2_values = 0:0.02:upper_bound;

[V1_ V2_] = meshgrid(V1_values(:), V2_values(:));
base_sub = 'Saved_Data';
specific_sub = 'Mixture_Models'; %'Gaussian_MovTgt' %'Gaussian_FixedTgt' Gaussian_MovTgt;
                                                   %circular, random,
                                                   %const_vel
mtx_file_name = 'fixed_tgt_gmm_random_MSE_PF';

%mixture_const_vel_EKF V1 = 0.05, V2 = 0, MSE = 21.18
%mixture_const_vel_PF 0.2, V2 = 0.05, MSE 34.72
%mixture_circular_EKF v1 = 0.05, V2 = 0, MSE 101.39
%m_circular_PF 0.55, 1.1, 95.51
%m_random_EKF 1.05, 0.55, 9.32
%m_random_PF 0.15, 0.25, 5.82


%Gaussian_Fixed_const_vel_ekf V1 =0.4, V2 = .25, MSE = 1.5878
%Gaussian_Fixed_const_vel_pf V1 =0.3, V2 = .15, MSE = 1.5090
%Gaussian_Fixed_cricular_ekf V1 =0, V2 = .15, MSE = 83.2481
%Gaussian_Fixed_circular_pf V1 = 0.5, V2 = 0.45, MSE = 82.3080
%Gaussian_Fixed_random_ekf V1 = 0.95, V2 = .75, MSE = 0.515
%Gaussian_Fixed_random_pf V1 = 0.1, V2 = .1, MSE = 0.804

%Gaussian_MovTgt_const_vel_ekf V1 = 0.9 V2 = 0 mSE = 0.8411
%Gaussian_MovTgt_const_vel_pf V1 = 0.5 V2 = 0.05 mSE = 80.081
%%Gaussian_MovTgt_circular_ekf V1 = 0.85, V2 = 1.95, MSE = 0.8862
%Gaussian_MovTgt_circular_pf V1 = 0.25, V2 = 0.3, MSE = 58.7761
%%Gaussian_MovTgt_random_ekf V1 = 1.95, V2 = 1.95, MSE = 0.8573
%%Gaussian_MovTgt_random_pf V1 = 0.05, V2 = 0.3, MSE = 63.771

mtx = readmtx(fullfile(base_sub,specific_sub,mtx_file_name),size(V1_,1),size(V1_,2),'float64')

figure
x = [];
size(V1_,1);
size(V1_,2);
fig = mesh(V1_,V2_,mtx,'FaceColor','none');
%scatter3(V1_,V2_,mtx)
xlabel('V1');
ylabel('V2');
zlabel('MSE');
%contour(V1_,V2_,mtx,30,'ShowText','on')
saveas(fig,fullfile(base_sub,specific_sub,'optimal_values',join([mtx_file_name,'.png'])));

mtx(mtx == -1) = 1.e1000; %if PF not converged, avoid it results a minimum!
'min'
minMatrix = min(mtx(:))
if minMatrix ~= 1.e1000
    index = mtx==minMatrix;
    [row,col] = find(index);
    'V1'
    V1_(row,col)
    'V2'
    V2_(row,col)
else
    'non converge mai'
end

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
%Gaussian_MovTgt
   %const_vel
        %EKF
            %optim MSE 1.9208
            %V1 = 1.2500, V2 = 0.6000
        %PF
            %optim PF 76.7531
            %v1 = 0.1, v2 = 0.45
    %circular
        %EKF
            %optim MSE 1.3853
            %V1 = 0.15, V2 = 1.15
        %PF
            %optim MSE 62.2229
            %V1 = 0.2, V2 = 1
    %random
        %EKF
            %optim MSE 2.9916
            %V1 = 1.3, V2 = 1.5
        %PF
            %optim MSE 86.7254
            %V1 = 0.1, V2 = 0.5
        
%fare un plot dall'alto con le curve di livello, magari è più sensato

%{
A = [0 1; 1 0];
save_matrix_to_file('Prova',A)
mtx = readmtx('Prova',2,2,'float64')
%}