function visualize_dots_plane(filt_x,target_traj,name,base_sub,specific_sub,experiment)
f = figure('Name',name);
plot(filt_x(1,:),filt_x(2,:),'x',target_traj(1,:),target_traj(2,:),'ro')
legend('Estimated','True')
saveas(f,fullfile(base_sub,specific_sub,experiment,join([name,'.png'])))
waitfor(f)
end

