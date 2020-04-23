function [MSEs]= FUNCTION_full_test(varargin)
%FUNCTION_FULL_TEST Execute the full test suite and return the MSEs
%  

[test_name,x,u,F,G,v1,v2,mu_x0,EKF_P1,PF_P1,EKF_V1,EKF_V2,PF_V1,PF_V2,mu_v2,time,endT,numparticles,mode] = varargin{1:19};

if strcmp(mode(2:end),'gaussian')
    visualize = varargin{20};
    if visualize == true
        base_sub = varargin{21};
        specific_sub = varargin{22};
        experiment = varargin{23};
    end
elseif strcmp(mode(2:end),'gaussian_mixture')
    visualize = varargin{20};
    if visualize == true
        base_sub = varargin{21};
        specific_sub = varargin{22};
        experiment = varargin{23};
    end
end

if visualize == true
f = msgbox(join(['Starting the ',test_name]), test_name,'warn');
waitfor(f)
end

if strcmp(mode(1),'0') %if still target
    if visualize == true
        [determ_x, determ_x_global, determ_y, stoch_x,stoch_x_global, stoch_y,y,target_trajectory] = FUNCTION_prepare_xy(x,u,F,G,v1,v2,time,endT,false,false,true,visualize,base_sub,specific_sub,experiment, test_name);
    else
        [determ_x, determ_x_global, determ_y, stoch_x,stoch_x_global, stoch_y,y,target_trajectory] = FUNCTION_prepare_xy(x,u,F,G,v1,v2,time,endT,false,false,true,visualize);
    end
elseif strcmp(mode(1),'1') % if moving target
    if visualize == true
        [determ_x, determ_x_global, determ_y, stoch_x,stoch_x_global, stoch_y,y,target_trajectory]  = FUNCTION_prepare_movtgt(x,u,F,G,v1,v2,time,endT,false,false,true,mu_x0(3:4,:),visualize,base_sub,specific_sub,experiment, test_name);
    else
        [determ_x, determ_x_global, determ_y, stoch_x,stoch_x_global, stoch_y,y,target_trajectory]  = FUNCTION_prepare_movtgt(x,u,F,G,v1,v2,time,endT,false,false,true,mu_x0(3:4,:),visualize);
    end
end

V2 = EKF_V2;
V1 = EKF_V1;
P1 = EKF_P1;
%EKF
if visualize == true
    [filt_x,filt_x_global] = FUNCTION_test_ekf(F,G,u,P1,V1,y,V2,mu_x0,target_trajectory,time,endT,determ_x,determ_x_global,stoch_x,stoch_x_global,visualize,base_sub,specific_sub,experiment, test_name);
else
    [filt_x,filt_x_global] = FUNCTION_test_ekf(F,G,u,P1,V1,y,V2,mu_x0,target_trajectory,time,endT,determ_x,determ_x_global,stoch_x,stoch_x_global,visualize);
end



% Particle filter
try %prevent crash
    if strcmp(mode(2:end),'gaussian')
	V2 = PF_V2;
	V1 = PF_V1;
    P1 = PF_P1;
        if strcmp(mode(1),'1')
            %V2 = 3
            if visualize == true
              [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,V1,mu_x0,P1,mu_v2,V2,visualize,base_sub,specific_sub,experiment, test_name);
            else
              [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,V1,mu_x0,P1,mu_v2,V2,visualize);
            end
        elseif strcmp(mode(1),'0')
            if visualize == true
              [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,V1,mu_x0,P1,mu_v2,V2,visualize,base_sub,specific_sub,experiment, test_name);
            else
              [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,V1,mu_x0,P1,mu_v2,V2,visualize);
            end
        end
    elseif strcmp(mode(2:end),'gaussian_mixture')
        if visualize == true
            gm_v1 = varargin{24};
            gm_v2 = varargin{25};
            [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian_mixture',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,gm_v1,mu_x0,P1,gm_v2,visualize,base_sub,specific_sub,experiment, test_name);
        else
            gm_v1 = varargin{21};
            gm_v2 = varargin{22};
            [part_filt_x,part_filt_x_global] = FUNCTION_test_pf(numparticles,'gaussian_mixture',F,G,y,u,time,endT,target_trajectory,determ_x,determ_x_global,stoch_x,stoch_x_global,gm_v1,mu_x0,P1,gm_v2,visualize);
        end
    end
    
    if visualize == true
    %comparison
        x1_cell = {filt_x(1,1:end-1),part_filt_x(1,1:end-1),stoch_x(1,1:endT),determ_x(1,1:endT)};
        x2_cell = {filt_x(2,1:end-1),part_filt_x(2,1:end-1),stoch_x(2,1:endT),determ_x(2,1:endT)};
        plot_multiple(2, join([test_name,'Comparison'],'_'), time,x1_cell,x2_cell,{'ekf','pf','stoch','determ'},base_sub,specific_sub,experiment, test_name)
        x1_cell = {filt_x_global(1,1:end-1),part_filt_x_global(1,1:end-1),stoch_x_global(1,1:endT),determ_x_global(1,1:endT)};
        x2_cell = {filt_x_global(2,1:end-1),part_filt_x_global(2,1:end-1),stoch_x_global(2,1:endT),determ_x_global(2,1:endT)};
        plot_multiple(2, join([test_name,'ComparisonGlobal'],'_'), time,x1_cell,x2_cell,{'ekfglobal','pfglobal','stochglobal','determglobal'},base_sub,specific_sub,experiment, test_name)

    end 
    %Compute MSE
    [filt_MSE, part_MSE] = FUNCTION_print_MSE(filt_x,part_filt_x,stoch_x,visualize);
    MSEs = [filt_MSE;part_MSE]

catch exception
   msgText = getReport(exception)
   part_filt_x = [];
   part_filt_x_global = [];
   if visualize == true
       x1_cell = {filt_x(1,1:end-1),stoch_x(1,1:endT),determ_x(1,1:endT)};
       x2_cell = {filt_x(2,1:end-1),stoch_x(2,1:endT),determ_x(2,1:endT)};
       plot_multiple(2, join([test_name,'Comparison'],'_'), time,x1_cell,x2_cell,{'filt','stoch','determ'},base_sub,specific_sub,experiment, test_name)
       x1_cell = {filt_x_global(1,1:end-1),stoch_x_global(1,1:endT),determ_x_global(1,1:endT)};
       x2_cell = {filt_x_global(2,1:end-1),stoch_x_global(2,1:endT),determ_x_global(2,1:endT)};
       plot_multiple(2, join([test_name,'ComparisonGlobal'],'_'), time,x1_cell,x2_cell,{'ekfglobal','pfglobal','stochglobal','determglobal'},base_sub,specific_sub,experiment, test_name)

   end
   [filt_MSE, part_MSE] = FUNCTION_print_MSE(filt_x,part_filt_x,stoch_x,visualize);
   MSEs = [filt_MSE;part_MSE]
end
end

