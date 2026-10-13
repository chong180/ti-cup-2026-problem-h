%% AB20 Ball Balance Tuner with CSV Export
%% Target: ball at 0mm, tolerance +-8mm
%% Frame: ,<pos>,<vel>,<ang>,<ff>,<gain>,<dist>,<pwmL>,<pwmR>,<state>,
%% Cmd: ,PG,<value>
clear; clc; close all;

COM_PORT = 'COM12';
BAUD = 115200;
WIN = 500;
TOL = 8.0;
pg = 0.35;

try
    s = serialport(COM_PORT, BAUD); flush(s);
    fprintf('Serial opened\n');
catch ME
    fprintf('Fail: %s\n', ME.message); return;
end

fig = figure('Name','AB20 Ball Tune','Position',[100 100 900 750],'CloseRequestFcn',@cleanup);

ax1=subplot(3,1,1); hold on; grid on;
title('Ball Position (mm)'); ylabel('mm');
lp=plot(0,0,'b-','LineWidth',1.5);
plot([-WIN 0],[0 0],'g--');
plot([-WIN 0],[TOL TOL],'r:');
plot([-WIN 0],[-TOL -TOL],'r:');
xlim([-WIN 0]); ylim([-60 60]);

ax2=subplot(3,1,2); hold on; grid on;
title('Ball Velocity (mm/s)'); ylabel('mm/s');
lv=plot(0,0,'m-','LineWidth',1.5);
xlim([-WIN 0]); ylim([-120 120]);

ax3=subplot(3,1,3); hold on; grid on;
title('Motor Angle (deg)'); xlabel('Sample'); ylabel('deg');
la=plot(0,0,'c-','LineWidth',1.5);
xlim([-WIN 0]); ylim([-15 15]); 

txt=uicontrol('style','text','pos',[10 700 880 25],'string',sprintf('PosGain=%.4f',pg));
uicontrol('style','pushbutton','pos',[10 660 60 30],'string','PG+','callback',@(~,~)adj(0.01));
uicontrol('style','pushbutton','pos',[80 660 60 30],'string','PG-','callback',@(~,~)adj(-0.01));
uicontrol('style','pushbutton','pos',[150 660 60 30],'string','PG+.05','callback',@(~,~)adj(0.05));
uicontrol('style','pushbutton','pos',[220 660 60 30],'string','PG-.05','callback',@(~,~)adj(-0.05));
pe=uicontrol('style','edit','pos',[290 660 100 30],'string',num2str(pg,'%.4f'));
uicontrol('style','pushbutton','pos',[400 660 60 30],'string','Apply','callback',@applyEdit);
uicontrol('style','pushbutton','pos',[470 660 90 30],'string','Export CSV','callback',@exportCSV,'BackgroundColor',[0.2 0.4 0.8],'ForegroundColor','w');

pb=zeros(1,WIN); vb=zeros(1,WIN); ab=zeros(1,WIN);
idx=0; rxb=''; t0=tic;
fprintf('Receiving...\n');

while isvalid(fig)
    if s.NumBytesAvailable>0
        d2=read(s,s.NumBytesAvailable,'char'); rxb=[rxb,d2];
        while contains(rxb,newline)
            nl=find(rxb==newline,1); if isempty(nl), break; end
            line=strtrim(rxb(1:nl-1)); rxb=rxb(nl+1:end);
            if strlength(line)<8 || ~startsWith(line,'$B,'), continue; end
            f=strrep(line(4:end),',',' ');
            vals=sscanf(f,'%g');
            if numel(vals)<9, continue; end
            idx=idx+1;
            pb=circshift(pb,-1); vb=circshift(vb,-1); ab=circshift(ab,-1);
            pb(end)=vals(1); vb(end)=vals(2); ab(end)=vals(3);
            xa=1:WIN;
            set(lp,'XData',xa,'YData',pb);set(lv,'XData',xa,'YData',vb);set(la,'XData',xa,'YData',ab);
            pmax=max(abs(pb));
            if pmax>50, ylim(ax1,[-pmax*1.2 pmax*1.2]); else ylim(ax1,[-60 60]); end
            drawnow;
        end
    else pause(0.002); end
end

function adj(delta)
    global pg pe txt s
    pg=max(0.05,min(1.0,pg+delta));
    set(pe,'string',num2str(pg,'%.4f'));
    set(txt,'string',sprintf('PosGain=%.4f',pg));
    write(s,sprintf('$C,PG,%.4f\n',pg),'char');
    fprintf('Sent PG=%.4f\n',pg);
end

function applyEdit(~,~)
    global pg pe txt s
    v=str2double(get(pe,'string'));
    if isnan(v), return; end
    pg=max(0.05,min(1.0,v));
    set(txt,'string',sprintf('PosGain=%.4f',pg));
    write(s,sprintf('$C,PG,%.4f\n',pg),'char');
    fprintf('Sent PG=%.4f\n',pg);
end

function exportCSV(~,~)
    global pb vb ab idx t0
    if idx<2, fprintf('No data\n'); return; end
    [fn,fp]=uiputfile('ab20_data.csv','Save CSV');
    if isequal(fn,0), return; end
    fid=fopen(fullfile(fp,fn),'w');
    fprintf(fid,'time_s,ball_position_mm,ball_velocity_mm_s,motor_angle_deg,ball_visible\n');
    for i=1:idx
        fprintf(fid,'%.3f,%.2f,%.2f,%.2f,1\n',toc(t0),pb(i),vb(i),ab(i));
    end
    fclose(fid);
    fprintf('Exported %d rows to %s\n',idx,fullfile(fp,fn));
end

function cleanup(~,~)
    try
        if isvalid(s), flush(s); close(s); end
    catch
    end
    fprintf('\nSerial closed.\n');
end
