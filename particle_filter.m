function part_filt_x = particle_filter(varargin)
%PARTICLE_FILTER Applies the particle filtering algorithm
%can be called with 'gaussian' or 'gaussian_mixture' mode (passed as first
%argument.

mode = varargin{1};
num_particles = varargin{2};
endT = varargin{3};
F = varargin{4};
G = varargin{5};
y = varargin{6};
u = varargin{7};
%Initialization
if strcmp(mode,'gaussian')
    V1 = varargin{8};
    mu_x0 = varargin{9};
    P1 = varargin{10};
    mu_v2 = varargin{11};
    V2  = varargin{12};
    temp = randn(size(F,2),num_particles);
    particles = P1*temp + mu_x0; %initial state gaussian assumption
    % moving target is addressed by proper
    %initialization values passed to the function
elseif strcmp(mode,'gaussian_mixture')
   gm_v1 = varargin{8};
   mu_x0 = varargin{9};
   P1 = varargin{10};
   gm_v2 = varargin{11};
   temp = randn(size(F,2),num_particles);
   particles = P1*temp + mu_x0; %initial state gaussian assumption (is not related to the gaussian mixture assumption!)   
end
err = zeros(1,num_particles);
part_filt_x = zeros(size(F,2),endT+1); %x(t|t)
part_filt_x(:,1) = mean(particles,2);

for i = 2:1:endT
    %PREDICTION
        %System propagation
        for j = 1:1:num_particles
            if strcmp(mode,'gaussian')
                v1_particle = V1*randn(size(F,2),1);
            elseif strcmp(mode,'gaussian_mixture')
                v1_particle = random(gm_v1,1).';
            end
            particles(:,j) = F*particles(:,j) + G*u(:,i-1)+v1_particle;%v1(i-1);
        end
        %visualize_dots_plane(particles,repmat([4;8],size(particles,2)),'test show particles')
        %'propagation'
        %unique(particles)
     %UPDATE
        %weight computation
        for j = 1:1:num_particles
            err(j) = y(i) - sqrt(particles(1,j)^2 + particles(2,j)^2);
        end
        
        if strcmp(mode,'gaussian')
            %mu_v2 = 0;
            w = normpdf(err,mu_v2,V2); %w = mvnpdf(x.',MU,SIGMA) %mvnpdf works by rows, hence .' on particles, anche il pi� generale pdf() pu� essere utile
            %err
            %w
        elseif strcmp(mode,'gaussian_mixture')
            w = pdf(gm_v2,err.').';
            %'print weighting with gaussian mixture'
            %w
        end
        %weight normalization
        w = w ./ sum(w);
     %ESTIMATION
        part_filt_x(:,i) = w*particles.';
        
     %RESAMPLING
         resampling_indexes = systematic_resampling(w);
         particles = particles(:,resampling_indexes);
         %'resampling'
         %unique(particles)
end

end

