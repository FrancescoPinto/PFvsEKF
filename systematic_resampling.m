function sample_indexes = systematic_resampling(w)
%SYSTEMATIC_RESAMPLING
%   Performs the importance resampling step by applying the systematic
%   resampling algorithm
%compute the CDF using the weights
w_cum = cumsum(w);
M = length(w);
%generate N ordered numbers with (3.69)
u = ([0:M-1]+rand(1))/M;
sample_indexes = zeros(1,M);
%take particle j a number of times equal to the
%number of elements in u with weight between q(j-1) and q(j)
i =1;  
j = 1; 
while (i <= M)
    if(u(i) < w_cum(j))
        sample_indexes(i) = j;
        i = i+1;
    else
        j = j+1;
    end
end
end

