function visualize_plane(x1,y1,x2,y2,x3,y3,name, base_sub,specific_sub,experiment)
%modification of the Comet method to visualize results
length(x1)
length(x2)
length(x3)
assert(length(x1) == length(y1) && length(x2) == length(y2) && length(x3) == length(y3) && length(x1) == length(x2) && length(x1) == length(x3))

pause_time = 0.0025
p = 0.10;
fig = figure('Name',name);
minx = min([min(x1) min(x2) min(x3)]);
maxx = max([max(x1) max(x2) max(x3)]);
miny = min([min(y1) min(y2) min(y3)]);
maxy = max([max(y1) max(y2) max(y3)]);
%[minx,maxx] = minmax(x);
%[miny,maxy] = minmax(y);
ax = subplot(1,1,1);
axis(ax,[minx maxx miny maxy]);

m = length(x1);
k = round(p*m);

co = get(ax,'colororder');
if size(co,1)>=3
    colors = [ co(1,:);co(2,:);co(3,:)];
    lstyle = '-';
else
    colors = repmat(co(1,:),3,1);
    lstyle = '--';
end

head_x1 = line('parent',ax,'color',colors(1,:),'marker','o','linestyle','none', ...
            'xdata',x1(1),'ydata',y1(1),'Tag','head');

body_x1 = matlab.graphics.animation.AnimatedLine('color',colors(2,:),...
    'linestyle',lstyle,...
    'Parent',ax,...
    'MaximumNumPoints',max(1,k),'tag','body');
tail_x1 = matlab.graphics.animation.AnimatedLine('color',colors(3,:),...
    'linestyle','-',...
    'Parent',ax,...
    'MaximumNumPoints',1+m,'tag','tail'); %Add 1 for any extra points

head_x2 = line('parent',ax,'color',colors(1,:),'marker','+','linestyle','none', ...
            'xdata',x1(1),'ydata',y1(1),'Tag','head');

body_x2 = matlab.graphics.animation.AnimatedLine('color','black',...
    'linestyle',lstyle,...
    'Parent',ax,...
    'MaximumNumPoints',max(1,k),'tag','body');
tail_x2 = matlab.graphics.animation.AnimatedLine('color','yellow',...
    'linestyle','-',...
    'Parent',ax,...
    'MaximumNumPoints',1+m,'tag','tail'); %Add 1 for any extra points

head_x3 = line('parent',ax,'color',colors(1,:),'marker','^','linestyle','none', ...
            'xdata',x1(1),'ydata',y1(1),'Tag','head');

body_x3 = matlab.graphics.animation.AnimatedLine('color','blue',...
    'linestyle',lstyle,...
    'Parent',ax,...
    'MaximumNumPoints',max(1,k),'tag','body');
tail_x3 = matlab.graphics.animation.AnimatedLine('color','green',...
    'linestyle','-',...
    'Parent',ax,...
    'MaximumNumPoints',1+m,'tag','tail'); %Add 1 for any extra points


if ( length(x1) < 2000 )
    updateFcn = @()drawnow;
else
    updateFcn = @()drawnow('update');
end

% Grow the body
%INIZI DAL FOGLIO BIANCO
%inserisci i primi K punti (che appartengono al "body" (ovvero la parte
%rossa, "pi� recente")
for i = 1:k
    set(head_x1,'xdata',x1(i),'ydata',y1(i));
    set(head_x2,'xdata',x2(i),'ydata',y2(i));
    set(head_x3,'xdata',x3(i),'ydata',y3(i));

    if  ~( body_x1.isvalid() )
        return
    end
    addpoints(body_x1,x1(i),y1(i)); %aggiungi punti all'animazione
    addpoints(body_x2,x2(i),y2(i));
    addpoints(body_x3,x3(i),y3(i));
    updateFcn();
    %waitforbuttonpress;
    pause(pause_time);
end
% Add a drawnow to capture any events / callbacks
drawnow;
%una volta che hai disegnato i primi K punti, devi disegnare i successivi
%come body, e quelli pi� vecchi di K li devi ricolorare per marcare che
%sono parte della coda (quindi fai addpoints sia su tail che coda)
% Primary loop
for i = k+1:m
    set(head_x1,'xdata',x1(i),'ydata',y1(i));
    set(head_x2,'xdata',x2(i),'ydata',y2(i));
    set(head_x3,'xdata',x3(i),'ydata',y3(i));
    if ~( body_x1.isvalid() )
        return
    end
    addpoints(tail_x1,x1(i-k),y1(i-k));
    addpoints(tail_x2,x2(i-k),y2(i-k));
    addpoints(tail_x3,x3(i-k),y3(i-k));
    addpoints(body_x1,x1(i),y1(i));
    addpoints(body_x2,x2(i),y2(i));
    addpoints(body_x3,x3(i),y3(i));

    updateFcn();
    %waitforbuttonpress;
    pause(pause_time);
end
drawnow;
% Clean up the tail
%col passare del tempo la "testa" si raffredda, quindi
%vai ad aggiungere alla coda tutti i rimanenti punti
for i = m+1:m+k
    if  ~( body_x1.isvalid() )
        return
    end
    addpoints(tail_x1,x1(i-k),y1(i-k));
    addpoints(tail_x2,x2(i-k),y2(i-k));
    addpoints(tail_x3,x3(i-k),y3(i-k));
    updateFcn();
end
drawnow;
saveas(fig,fullfile(base_sub,specific_sub,experiment,join([name,'.png'])))
waitfor(fig)
end






