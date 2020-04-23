function plot_multiple( varargin )
%PLOT_MULTIPLE Helper function, allows to easily plot multiple data on the
%same plot
num_subplots = varargin{1};
name = varargin{2};
time = varargin{3};
data_multiple_1 = varargin{4};
data_multiple_2 = varargin{5};
if num_subplots == 3
    data_multiple_3 = varargin{6};
    legends = varargin{7};
    base_sub = varargin{8};
    specific_sub = varargin{9};
    experiment = varargin{10};
elseif num_subplots == 2
    legends = varargin{6};
    base_sub = varargin{7};
    specific_sub = varargin{8};
    experiment = varargin{9};
end

num_lines = size(legends,2);
m = figure('Name',name);
subplot(num_subplots,1,1);
for i = 1:1:num_lines
    %i
    %size(time)
    %size(data_multiple_1{i})
    plot(time,data_multiple_1{i})
    hold on
end
ylabel('xtgt')
legend(legends,'location','eastoutside')

subplot(num_subplots,1,2);
for i = 1:1:num_lines
    plot(time,data_multiple_2{i})
    hold on
end
ylabel('ytgt')
legend(legends,'location','eastoutside')

if num_subplots == 3
%aggiunge la parte della distance 
subplot(num_subplots,1,3);
for i = 1:1:2
    plot(time,data_multiple_3{i})
    hold on
end
ylabel('distance')
legend(legends(1,1:2),'location','eastoutside')
end
saveas(m,fullfile(base_sub,specific_sub,experiment,join([name,'.png'])))
waitfor(m)

end

