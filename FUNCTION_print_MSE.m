function [MSE_filt_x,MSE_part_filt_x] = FUNCTION_print_MSE(filt_x, part_filt_x, stoch_x,visualize)
%FUNCTION_PRINT_MSE Compute the MSE of the two filters (if possible) and
%plot it (if required)
%   Detailed explanation goes here

diff = (filt_x - stoch_x).^2;
summation = sum(sum(diff));
MSE_filt_x = summation/size(filt_x,2);

if size(part_filt_x) ~= 0
    diff = (part_filt_x - stoch_x).^2;
    summation = sum(sum(diff));
    MSE_part_filt_x = summation/size(filt_x,2);
else
    MSE_part_filt_x = -1; %if the program crashed, set MSE to impossible value!
end



if visualize == true
f = msgbox(sprintf('EKF MSE: \n %f \n  PF MSE: \n %f ',MSE_filt_x,MSE_part_filt_x), 'MSE Evaluation','warn');
waitfor(f)
end


end

