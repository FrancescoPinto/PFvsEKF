function [filt_x,e] = extended_kalman_filter( endT, F,G,u,P1,V1,y,V2,mu_x0)
%EXTENDED_KALMAN_FILTER Implementation of the extended kalman filter
%   Detailed explanation goes here
'mu_x0'
mu_x0
num_state_variables = length(mu_x0);
%PREDICTION (Initialization)
pred_x = zeros(num_state_variables,endT+1); %x(t|t-1)
filt_x = zeros(num_state_variables,endT+1); %x(t|t)
temp = randn(num_state_variables,1);

'P1temp'
P1*temp
filt_x(:,1)  = mu_x0;%x(1) ~ G(mu_x0,P1)
'filt_x init'
filt_x(:,1)

pred_x(:,2) = F*filt_x(:,1)+G*u(:,2); %x(2|1)
filt_P = P1; %P(1|1)
pred_P = F*filt_P*F.'+V1;%P(2|1)

for i = 2:1:endT
        %UPDATE
        e = y(i) - sqrt(pred_x(1,i)^2 + pred_x(2,i)^2)
        H = [pred_x(1,i)/sqrt(pred_x(1,i).^2+pred_x(2,i)^2), pred_x(2,i)/sqrt(pred_x(1,i).^2+pred_x(2,i)^2)];
        %pad with zeros
        if num_state_variables > 2
         H = [H,0,0]; %constant velocity case, derivative of h(x) ... but h independent of velocities
        end
        S = H*pred_P*H.' + V2;
        K = pred_P*H.'/S;
        filt_x(:,i) = pred_x(:,i)+K*e;
        filt_P = (eye(num_state_variables)-K*H)*pred_P;
        
        %PREDICTION
        %'trying u'
        %size(u)
        %'trying x'
        %size(filt_x)
        pred_x(:,i+1) = F*filt_x(:,i)+G*u(:,i+1);
        pred_P = F*filt_P*F.' + V1;
        
end 
end

