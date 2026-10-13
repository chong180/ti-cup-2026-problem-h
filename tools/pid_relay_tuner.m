function PID_Tuner()
% PID_Tuner.m — 无线串口实时PID调参 & 遥控 & 编码器调试 & 自整定工具 (重新设计版)
%
% 图表说明:
%   左上: 循迹误差曲线 — 误差=0表示在线中心
%   右上: 实测轮速曲线 — 绿=左轮, 橙=右轮, 蓝虚=平均速度 (cm/s, 编码器实测)
%   左下: 误差阶跃响应 — 参数修改后的误差收敛过程 (真实时间轴)
%   右下: 2D运动轨迹 — 编码器里程 + 陀螺仪航向 航位推算
%
% 自整定 (继电反馈法, 重做):
%   1. MCU切换到bang-bang控制(±35), 系统产生极限环振荡
%   2. MATLAB用滞回过零检测测量振荡周期Tu和振幅a (抗量化噪声)
%   3. 振荡规则性校验 (周期/振幅变异系数), 不规则则拒绝给参数
%   4. 提供4套整定规则 (经典ZN/微超调/无超调/Tyreus-Luyben), 任选下发
%
% 通信协议:
%   MCU->PC: $T,<yaw>,<err>,<L%>,<R%>,<kp>,<ki>,<kd>,<mode>,<state>,
%             <mask>,<spd>,<vL>,<vR>,<dL>,<dR>\r\n   (15字段)
%   PC->MCU: $C,<cmd>[,<param>]\r\n
%   调试:   ET,L/ET,R/ET,S (单轮编码器测试)  DCLR (里程清零)

clc; close all;

%% ======================== 串口连接 ========================
ports = serialportlist;
fprintf('可用串口: %s\n', strjoin(ports, ', '));
if numel(ports) == 1
    comPort = ports(1);
    fprintf('仅一个串口, 自动选择 %s\n', comPort);
else
    comPort = string(input('请输入串口号 (如 COM3, 回车用第一个): ', 's'));
    if strlength(comPort) == 0
        comPort = ports(1);
        fprintf('使用 %s\n', comPort);
    end
end

global s_serial
try
    s_serial = serialport(comPort, 115200);
    configureTerminator(s_serial, "LF");
    flush(s_serial);
    fprintf('已连接 %s @ 115200 bps\n\n', comPort);
catch ME
    fprintf('串口打开失败: %s\n', ME.message);
    return;
end

%% ======================== 数据缓冲区 (半缓冲搬移, 不用circshift) ========================
MAX_PTS = 4000;
global g_data g_idx g_running g_t0
g_data = struct(...
    't',     zeros(1, MAX_PTS), 'yaw',   zeros(1, MAX_PTS), ...
    'err',   zeros(1, MAX_PTS), 'left',  zeros(1, MAX_PTS), ...
    'right', zeros(1, MAX_PTS), 'kp',    zeros(1, MAX_PTS), ...
    'ki',    zeros(1, MAX_PTS), 'kd',    zeros(1, MAX_PTS), ...
    'mode',  zeros(1, MAX_PTS), 'state', zeros(1, MAX_PTS), ...
    'mask',  zeros(1, MAX_PTS), 'speed', zeros(1, MAX_PTS), ...
    'vL',    zeros(1, MAX_PTS), 'vR',    zeros(1, MAX_PTS), ...
    'dL',    zeros(1, MAX_PTS), 'dR',    zeros(1, MAX_PTS));
g_idx = 0;
g_running = true;
g_t0 = tic;

% 阶跃响应: 记录参数修改后的 (真实时间, 误差)
global g_step
g_step = struct('t0', 0, 'ts', zeros(1, 500), 'vals', zeros(1, 500), 'n', 0);

% 遥控按住状态跟踪
global g_holdAction
g_holdAction = '';

% 2D轨迹 (编码器里程 + 陀螺仪航向)
global g_traj
TRAJ_MAX = 6000;
g_traj = struct('x', 0, 'y', 0, 'xs', zeros(1, TRAJ_MAX), ...
    'ys', zeros(1, TRAJ_MAX), 'n', 0, 'max', TRAJ_MAX, ...
    'prev_dL', NaN, 'prev_dR', NaN);

% 自动整定状态
global g_tune
g_tune = struct('active', false, 't0', 0, 'count', 0, ...
    'err_buf', zeros(1, 4000), 't_buf', zeros(1, 4000), 'halfcycles', 0);

%% ======================== 白色主题配色 ========================
BG       = [0.95 0.95 0.97];
PANEL_BG = [0.90 0.90 0.93];
AX_BG    = [1.0 1.0 1.0];
GRID_C   = [0.80 0.80 0.84];
TEXT_C   = [0.12 0.12 0.12];
GREEN    = [0.0 0.65 0.35];
ORANGE   = [0.85 0.50 0.0];
RED      = [0.80 0.18 0.18];
BLUE     = [0.10 0.45 0.85];
GREY     = [0.45 0.45 0.48];
BTN_GREEN = [0.18 0.60 0.28];
BTN_RED   = [0.72 0.16 0.16];
BTN_BLUE  = [0.14 0.40 0.72];
BTN_ORANGE= [0.80 0.48 0.0];
BTN_GREY  = [0.50 0.50 0.53];
BTN_PURPLE= [0.40 0.25 0.60];

%% ======================== 创建GUI窗口 ========================
fig = figure('Name', 'PID Tuner — 调参 & 遥控 & 编码器调试 & 自整定', ...
    'NumberTitle', 'off', 'MenuBar', 'none', ...
    'Position', [40 30 1500 950], 'Color', BG);
fig.CloseRequestFcn = @onClose;
fig.WindowButtonUpFcn = @onMouseUp;

%% ======================== 顶部状态栏 ========================
uicontrol(fig,'Style','text','String','PID Tuner',...
    'Position',[15 915 110 30],'FontSize',15,'FontWeight','bold',...
    'BackgroundColor',BG,'ForegroundColor',BLUE);

global h_pidtext
h_pidtext = uicontrol(fig,'Style','text',...
    'String','Kp=---  Ki=---  Kd=---  SPD=---',...
    'Position',[130 915 330 30],'FontSize',11,'FontWeight','bold',...
    'BackgroundColor',PANEL_BG,'ForegroundColor',TEXT_C);

uicontrol(fig,'Style','text','String','模式:',...
    'Position',[470 917 35 26],'FontSize',10,...
    'BackgroundColor',BG,'ForegroundColor',TEXT_C);
global h_mode
h_mode = uicontrol(fig,'Style','popupmenu',...
    'String',{'TRACK','A->B','ABCD','ACBD','AC4','TRK50','REMOTE'},...
    'Position',[505 916 90 26],'FontSize',10,...
    'BackgroundColor','w','ForegroundColor',TEXT_C,'Callback',@onModeChange);

global h_modetext
h_modetext = uicontrol(fig,'Style','text','String','MASK=--',...
    'Position',[605 915 110 30],'FontSize',10,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',GREY);

global h_yawtext
h_yawtext = uicontrol(fig,'Style','text','String','Yaw=---',...
    'Position',[725 915 120 30],'FontSize',10,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',BLUE);

global h_veltext
h_veltext = uicontrol(fig,'Style','text','String','V=---',...
    'Position',[855 915 220 30],'FontSize',10,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',GREEN);

global h_errtext
h_errtext = uicontrol(fig,'Style','text','String','Err=---',...
    'Position',[1085 915 85 30],'FontSize',10,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',RED);

global h_tunetext
h_tunetext = uicontrol(fig,'Style','text','String','',...
    'Position',[1180 915 305 30],'FontSize',9,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',BTN_PURPLE);

%% ======================== 2x2 图表区域 ========================
% 左上: 循迹误差
ax_err = axes(fig,'Position',[0.02 0.56 0.46 0.34]);
global h_err_line
h_err_line = plot(ax_err, 0, 0, '-', 'Color', RED, 'LineWidth', 1.5);
hold(ax_err, 'on');
plot(ax_err, [0 30], [0 0], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
plot(ax_err, [0 30], [3 3], ':', 'Color', [0.3 0.6 0.3], 'LineWidth', 0.5);
plot(ax_err, [0 30], [-3 -3], ':', 'Color', [0.3 0.6 0.3], 'LineWidth', 0.5);
title(ax_err, '循迹误差  (0=在线中心  +:线偏右  -:线偏左)', 'Color', TEXT_C, 'FontSize', 10);
xlabel(ax_err, '时间 (s)', 'Color', TEXT_C);
ylabel(ax_err, 'Error', 'Color', TEXT_C);
set(ax_err,'Color',AX_BG,'XColor',TEXT_C,'YColor',TEXT_C,'GridColor',GRID_C,...
    'GridAlpha',0.6,'FontSize',9,'TickDir','out');
grid(ax_err,'on'); ylim(ax_err,[-8 8]);

% 右上: 实测轮速 (编码器)
ax_spd = axes(fig,'Position',[0.52 0.56 0.46 0.34]);
global h_vL_line h_vR_line h_vAvg_line
h_vL_line = plot(ax_spd, 0, 0, '-', 'Color', GREEN, 'LineWidth', 1.5);
hold(ax_spd, 'on');
h_vR_line = plot(ax_spd, 0, 0, '-', 'Color', ORANGE, 'LineWidth', 1.5);
h_vAvg_line = plot(ax_spd, 0, 0, '--', 'Color', BLUE, 'LineWidth', 1.2);
plot(ax_spd, [0 30], [0 0], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
title(ax_spd, '实测轮速  (绿=左轮  橙=右轮  蓝虚=平均速度)', 'Color', TEXT_C, 'FontSize', 10);
xlabel(ax_spd, '时间 (s)', 'Color', TEXT_C);
ylabel(ax_spd, '速度 (cm/s)', 'Color', TEXT_C);
legend(ax_spd, {'左轮','右轮','平均'}, 'Location','northeast',...
    'Color',PANEL_BG,'TextColor',TEXT_C,'FontSize',8);
set(ax_spd,'Color',AX_BG,'XColor',TEXT_C,'YColor',TEXT_C,'GridColor',GRID_C,...
    'GridAlpha',0.6,'FontSize',9,'TickDir','out');
grid(ax_spd,'on'); ylim(ax_spd,[-120 120]);

% 左下: 误差阶跃响应 (真实时间轴)
ax_step = axes(fig,'Position',[0.02 0.23 0.46 0.29]);
global h_step_line
h_step_line = plot(ax_step, 0, 0, '-', 'Color', BLUE, 'LineWidth', 1.5);
hold(ax_step, 'on');
plot(ax_step, [0 10], [0 0], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
title(ax_step, '阶跃响应  (改参数后误差收敛过程)', 'Color', TEXT_C, 'FontSize', 10);
xlabel(ax_step, '距上次调参 (s)', 'Color', TEXT_C);
ylabel(ax_step, 'Error', 'Color', TEXT_C);
set(ax_step,'Color',AX_BG,'XColor',TEXT_C,'YColor',TEXT_C,'GridColor',GRID_C,...
    'GridAlpha',0.6,'FontSize',9,'TickDir','out');
grid(ax_step,'on'); ylim(ax_step,[-8 8]); xlim(ax_step,[0 10]);

% 右下: 2D运动轨迹 (编码器里程 + 陀螺仪航向)
ax_traj = axes(fig,'Position',[0.52 0.23 0.46 0.29]);
global h_traj_line h_traj_dot h_traj_start
h_traj_line = plot(ax_traj, 0, 0, '-', 'Color', BLUE, 'LineWidth', 1.2);
hold(ax_traj, 'on');
h_traj_dot = plot(ax_traj, 0, 0, 'o', 'MarkerSize', 6, ...
    'MarkerFaceColor', RED, 'MarkerEdgeColor', RED);
h_traj_start = plot(ax_traj, 0, 0, 's', 'MarkerSize', 6, ...
    'MarkerFaceColor', GREEN, 'MarkerEdgeColor', GREEN);
title(ax_traj, '2D 运动轨迹  (编码器里程 + 陀螺仪航向)', 'Color', TEXT_C, 'FontSize', 10);
xlabel(ax_traj, 'X (cm)', 'Color', TEXT_C);
ylabel(ax_traj, 'Y (cm)', 'Color', TEXT_C);
set(ax_traj,'Color',AX_BG,'XColor',TEXT_C,'YColor',TEXT_C,'GridColor',GRID_C,...
    'GridAlpha',0.6,'FontSize',9,'TickDir','out');
grid(ax_traj,'on'); axis(ax_traj,'equal');
xlim(ax_traj,[-50 50]); ylim(ax_traj,[-50 50]);

%% ======================== 遥控面板 ========================
uicontrol(fig,'Style','text','String','遥 控 面 板',...
    'Position',[15 195 575 22],'FontSize',12,'FontWeight','bold',...
    'BackgroundColor',PANEL_BG,'ForegroundColor',TEXT_C,'HorizontalAlignment','center');

uicontrol(fig,'Style','text','String','转弯:',...
    'Position',[15 162 40 22],'FontSize',9,...
    'BackgroundColor',BG,'ForegroundColor',GREY);
uicontrol(fig,'Style','pushbutton','String','← 90°',...
    'Position',[60 160 90 28],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_ORANGE,'ForegroundColor','w','Callback',@onTL90);
uicontrol(fig,'Style','pushbutton','String','← 180°',...
    'Position',[155 160 90 28],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_ORANGE,'ForegroundColor','w','Callback',@onTL180);
uicontrol(fig,'Style','pushbutton','String','180° →',...
    'Position',[250 160 90 28],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_ORANGE,'ForegroundColor','w','Callback',@onTR180);
uicontrol(fig,'Style','pushbutton','String','90° →',...
    'Position',[345 160 90 28],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_ORANGE,'ForegroundColor','w','Callback',@onTR90);
uicontrol(fig,'Style','pushbutton','String','■ 停止',...
    'Position',[460 160 110 28],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_GREY,'ForegroundColor','w','Callback',@onRcStop);

uicontrol(fig,'Style','text','String','方向:',...
    'Position',[15 115 40 22],'FontSize',9,...
    'BackgroundColor',BG,'ForegroundColor',GREY);
uicontrol(fig,'Style','pushbutton','String','↺ 逆时针',...
    'Position',[60 110 95 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_BLUE,'ForegroundColor','w','Enable','inactive',...
    'ButtonDownFcn',@onCcwStart);
uicontrol(fig,'Style','pushbutton','String','▲ 前进',...
    'Position',[160 110 95 36],'FontSize',11,'FontWeight','bold',...
    'BackgroundColor',BTN_GREEN,'ForegroundColor','w','Enable','inactive',...
    'ButtonDownFcn',@onFwdStart);
uicontrol(fig,'Style','pushbutton','String','▼ 后退',...
    'Position',[260 110 95 36],'FontSize',11,'FontWeight','bold',...
    'BackgroundColor',BTN_RED,'ForegroundColor','w','Enable','inactive',...
    'ButtonDownFcn',@onBwdStart);
uicontrol(fig,'Style','pushbutton','String','顺时针 ↻',...
    'Position',[360 110 95 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_BLUE,'ForegroundColor','w','Enable','inactive',...
    'ButtonDownFcn',@onCwStart);
uicontrol(fig,'Style','pushbutton','String','■ 停止',...
    'Position',[465 110 105 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_GREY,'ForegroundColor','w','Callback',@onRcStop);

uicontrol(fig,'Style','text','String',...
    {'方向按钮: 鼠标按住=动作, 松开=停止   |   转弯按钮: 点击自动转到目标角度', ...
     '(需先切换到REMOTE模式)'},...
    'Position',[15 62 575 32],'FontSize',8,...
    'BackgroundColor',BG,'ForegroundColor',GREY,'HorizontalAlignment','center');

%% ======================== 自动整定面板 ========================
uicontrol(fig,'Style','text','String','自 动 整 定  (继电反馈法)',...
    'Position',[620 195 390 22],'FontSize',12,'FontWeight','bold',...
    'BackgroundColor',PANEL_BG,'ForegroundColor',TEXT_C,'HorizontalAlignment','center');

global h_autotune_btn
h_autotune_btn = uicontrol(fig,'Style','pushbutton','String','自动整定',...
    'Position',[640 110 160 45],'FontSize',12,'FontWeight','bold',...
    'BackgroundColor',BTN_PURPLE,'ForegroundColor','w','Callback',@onAutoTune);

uicontrol(fig,'Style','text','String',...
    {'启动后MCU以±35 bang-bang控制产生极限环振荡,', ...
     '滞回过零法测Tu/振幅, 振荡规则后给出4套规则参数', ...
     '★ 整定时请确保小车在赛道上且能看到黑线'},...
    'Position',[620 55 390 45],'FontSize',8,...
    'BackgroundColor',BG,'ForegroundColor',GREY,'HorizontalAlignment','center');

uicontrol(fig,'Style','pushbutton','String','清除曲线',...
    'Position',[820 110 160 45],'FontSize',12,'FontWeight','bold',...
    'BackgroundColor',BTN_GREY,'ForegroundColor','w','Callback',@onClear);

%% ======================== 编码器调试面板 (新增) ========================
uicontrol(fig,'Style','text','String','编 码 器 调 试',...
    'Position',[1030 195 455 22],'FontSize',12,'FontWeight','bold',...
    'BackgroundColor',PANEL_BG,'ForegroundColor',TEXT_C,'HorizontalAlignment','center');

% 单轮测试: 验证编码器方向与计数 (固件ET命令, 自动切REMOTE)
uicontrol(fig,'Style','pushbutton','String','左轮测试',...
    'Position',[1040 110 105 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',GREEN,'ForegroundColor','w',...
    'Callback',@onTestLeft,'TooltipString','仅左轮转(30%): 左轮速度应为正');
uicontrol(fig,'Style','pushbutton','String','右轮测试',...
    'Position',[1155 110 105 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',ORANGE,'ForegroundColor','w',...
    'Callback',@onTestRight,'TooltipString','仅右轮转(30%): 右轮速度应为正');
uicontrol(fig,'Style','pushbutton','String','测试停止',...
    'Position',[1270 110 100 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_GREY,'ForegroundColor','w','Callback',@onTestStop);

% 里程/轨迹/导出
uicontrol(fig,'Style','pushbutton','String','里程清零',...
    'Position',[1040 60 105 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_ORANGE,'ForegroundColor','w',...
    'Callback',@onDistClear,'TooltipString','编码器里程清零 + 轨迹原点重置');
uicontrol(fig,'Style','pushbutton','String','轨迹重置',...
    'Position',[1155 60 105 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_GREY,'ForegroundColor','w',...
    'Callback',@onTrajReset,'TooltipString','仅重置MATLAB端轨迹显示');
uicontrol(fig,'Style','pushbutton','String','导出数据',...
    'Position',[1270 60 100 36],'FontSize',10,'FontWeight','bold',...
    'BackgroundColor',BTN_BLUE,'ForegroundColor','w',...
    'Callback',@onExport,'TooltipString','导出缓冲数据到.mat文件');

global h_enctext
h_enctext = uicontrol(fig,'Style','text','String','D: L=--- R=--- cm',...
    'Position',[1380 60 100 80],'FontSize',8,...
    'BackgroundColor',PANEL_BG,'ForegroundColor',TEXT_C);

%% ======================== 底部 PID 控制面板 ========================
y1 = 8; h = 30; bw = 26; ew = 50; gap = 3;
x = 15;

mkGroup(fig, x, y1, 'Kp', GREEN, BG, TEXT_C);  x = x + 27;
uicontrol(fig,'Style','pushbutton','String','-','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_RED,'ForegroundColor','w','Callback',@onDecKp);
x = x + bw + gap;
global h_kp
h_kp = uicontrol(fig,'Style','edit','String','0.55','Position',[x y1 ew h],...
    'FontSize',11,'BackgroundColor','w','ForegroundColor',TEXT_C,...
    'KeyPressFcn',@onKpEnter,'TooltipString','输入Kp值后回车');
x = x + ew + gap;
uicontrol(fig,'Style','pushbutton','String','+','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_GREEN,'ForegroundColor','w','Callback',@onIncKp);
x = x + bw + gap;
uicontrol(fig,'Style','text','String','+0.05','Position',[x y1+2 32 h-4],...
    'FontSize',7,'BackgroundColor',BG,'ForegroundColor',GREY);
x = x + 37;

mkGroup(fig, x, y1, 'Ki', BLUE, BG, TEXT_C);  x = x + 27;
uicontrol(fig,'Style','pushbutton','String','-','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_RED,'ForegroundColor','w','Callback',@onDecKi);
x = x + bw + gap;
global h_ki
h_ki = uicontrol(fig,'Style','edit','String','0.00','Position',[x y1 ew h],...
    'FontSize',11,'BackgroundColor','w','ForegroundColor',TEXT_C,'KeyPressFcn',@onKiEnter);
x = x + ew + gap;
uicontrol(fig,'Style','pushbutton','String','+','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_GREEN,'ForegroundColor','w','Callback',@onIncKi);
x = x + bw + gap;
uicontrol(fig,'Style','text','String','+0.005','Position',[x y1+2 32 h-4],...
    'FontSize',7,'BackgroundColor',BG,'ForegroundColor',GREY);
x = x + 37;

mkGroup(fig, x, y1, 'Kd', RED, BG, TEXT_C);  x = x + 27;
uicontrol(fig,'Style','pushbutton','String','-','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_RED,'ForegroundColor','w','Callback',@onDecKd);
x = x + bw + gap;
global h_kd
h_kd = uicontrol(fig,'Style','edit','String','0.000','Position',[x y1 ew h],...
    'FontSize',11,'BackgroundColor','w','ForegroundColor',TEXT_C,'KeyPressFcn',@onKdEnter);
x = x + ew + gap;
uicontrol(fig,'Style','pushbutton','String','+','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_GREEN,'ForegroundColor','w','Callback',@onIncKd);
x = x + bw + gap;
uicontrol(fig,'Style','text','String','+0.001','Position',[x y1+2 32 h-4],...
    'FontSize',7,'BackgroundColor',BG,'ForegroundColor',GREY);
x = x + 37;

mkGroup(fig, x, y1, '车速', ORANGE, BG, TEXT_C);  x = x + 37;
uicontrol(fig,'Style','pushbutton','String','-','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_RED,'ForegroundColor','w','Callback',@onDecSpd);
x = x + bw + gap;
global h_spd
h_spd = uicontrol(fig,'Style','edit','String','25','Position',[x y1 ew h],...
    'FontSize',11,'BackgroundColor','w','ForegroundColor',TEXT_C,'KeyPressFcn',@onSpdEnter);
x = x + ew + gap;
uicontrol(fig,'Style','pushbutton','String','+','Position',[x y1 bw h],...
    'FontSize',13,'FontWeight','bold','BackgroundColor',BTN_GREEN,'ForegroundColor','w','Callback',@onIncSpd);
x = x + bw + gap;
uicontrol(fig,'Style','text','String','+5%','Position',[x y1+2 27 h-4],...
    'FontSize',7,'BackgroundColor',BG,'ForegroundColor',GREY);
x = x + 32;

uicontrol(fig,'Style','pushbutton','String','RUN','Position',[x y1 55 h],...
    'FontSize',11,'FontWeight','bold','BackgroundColor',BTN_GREEN,'ForegroundColor','w','Callback',@onRun);
x = x + 58;
uicontrol(fig,'Style','pushbutton','String','STOP','Position',[x y1 55 h],...
    'FontSize',11,'FontWeight','bold','BackgroundColor',BTN_RED,'ForegroundColor','w','Callback',@onStop);
x = x + 58;
uicontrol(fig,'Style','pushbutton','String','归零','Position',[x y1 50 h],...
    'FontSize',10,'BackgroundColor',BTN_ORANGE,'ForegroundColor','w',...
    'Callback',@onRst,'TooltipString','陀螺仪航向归零');
x = x + 53;
uicontrol(fig,'Style','pushbutton','String','清空','Position',[x y1 45 h],...
    'FontSize',9,'BackgroundColor',BTN_GREY,'ForegroundColor','w','Callback',@onClear);

%% ======================== 定时器 ========================
global rx_buf
rx_buf = '';

tmr = timer('ExecutionMode','fixedRate','Period',0.02,...
    'TimerFcn',{@onTimer, s_serial, g_t0},'BusyMode','drop');
start(tmr);

global g_timer
g_timer = tmr;

end  % PID_Tuner 主函数结束


% ======================== 辅助: 创建标签 ========================
function mkGroup(fig, x, y, label, color, bg, txtc)
    uicontrol(fig,'Style','text','String',label,'Position',[x y 24 30],...
        'FontSize',10,'FontWeight','bold','BackgroundColor',bg,'ForegroundColor',color);
end


% ======================== 发送命令 (内部工具) ========================
function sendCmd(cmd, param)
    global s_serial
    if isempty(param)
        frame = sprintf('$C,%s\r\n', cmd);
    else
        frame = sprintf('$C,%s,%s\r\n', cmd, param);
    end
    write(s_serial, frame, 'char');
    fprintf('发送: %s', frame);
end


% ======================== 触发阶跃响应记录 ========================
function triggerStep()
    global g_step g_t0
    g_step.t0 = toc(g_t0);
    g_step.n = 0;
end


% ======================== 模式切换 ========================
function onModeChange(~, ~)
    global h_mode
    idx = get(h_mode, 'Value') - 1;
    sendCmd('MODE', num2str(idx));
end


% ======================== Kp 调节 ========================
function onSetKp(~,~)
    global h_kp
    val = str2double(get(h_kp, 'String'));
    if isnan(val), return; end
    sendCmd('KP', sprintf('%.3f', val));
    triggerStep();
end

function onIncKp(~,~)
    global h_kp
    val = str2double(get(h_kp, 'String'));
    if isnan(val), val = 0; end
    val = val + 0.05;
    set(h_kp, 'String', sprintf('%.2f', val));
    sendCmd('KP', sprintf('%.3f', val));
    triggerStep();
end

function onDecKp(~,~)
    global h_kp
    val = str2double(get(h_kp, 'String'));
    if isnan(val), val = 0; end
    val = val - 0.05;
    set(h_kp, 'String', sprintf('%.2f', val));
    sendCmd('KP', sprintf('%.3f', val));
    triggerStep();
end

function onKpEnter(~, event)
    if strcmp(event.Key, 'return'), onSetKp(); end
end


% ======================== Ki 调节 ========================
function onSetKi(~,~)
    global h_ki
    val = str2double(get(h_ki, 'String'));
    if isnan(val), return; end
    sendCmd('KI', sprintf('%.4f', val));
    triggerStep();
end

function onIncKi(~,~)
    global h_ki
    val = str2double(get(h_ki, 'String'));
    if isnan(val), val = 0; end
    val = val + 0.005;
    set(h_ki, 'String', sprintf('%.3f', val));
    sendCmd('KI', sprintf('%.4f', val));
    triggerStep();
end

function onDecKi(~,~)
    global h_ki
    val = str2double(get(h_ki, 'String'));
    if isnan(val), val = 0; end
    val = val - 0.005;
    set(h_ki, 'String', sprintf('%.3f', val));
    sendCmd('KI', sprintf('%.4f', val));
    triggerStep();
end

function onKiEnter(~, event)
    if strcmp(event.Key, 'return'), onSetKi(); end
end


% ======================== Kd 调节 ========================
function onSetKd(~,~)
    global h_kd
    val = str2double(get(h_kd, 'String'));
    if isnan(val), return; end
    sendCmd('KD', sprintf('%.4f', val));
    triggerStep();
end

function onIncKd(~,~)
    global h_kd
    val = str2double(get(h_kd, 'String'));
    if isnan(val), val = 0; end
    val = val + 0.001;
    set(h_kd, 'String', sprintf('%.3f', val));
    sendCmd('KD', sprintf('%.4f', val));
    triggerStep();
end

function onDecKd(~,~)
    global h_kd
    val = str2double(get(h_kd, 'String'));
    if isnan(val), val = 0; end
    val = val - 0.001;
    set(h_kd, 'String', sprintf('%.3f', val));
    sendCmd('KD', sprintf('%.4f', val));
    triggerStep();
end

function onKdEnter(~, event)
    if strcmp(event.Key, 'return'), onSetKd(); end
end


% ======================== 车速 调节 ========================
function onSetSpd(~,~)
    global h_spd
    val = round(str2double(get(h_spd, 'String')));
    if isnan(val), return; end
    if val < 5, val = 5; end
    if val > 80, val = 80; end
    set(h_spd, 'String', num2str(val));
    sendCmd('SPD', num2str(val));
    triggerStep();
end

function onIncSpd(~,~)
    global h_spd
    val = round(str2double(get(h_spd, 'String')));
    if isnan(val), val = 25; end
    val = val + 5;
    if val > 80, val = 80; end
    set(h_spd, 'String', num2str(val));
    sendCmd('SPD', num2str(val));
    triggerStep();
end

function onDecSpd(~,~)
    global h_spd
    val = round(str2double(get(h_spd, 'String')));
    if isnan(val), val = 25; end
    val = val - 5;
    if val < 5, val = 5; end
    set(h_spd, 'String', num2str(val));
    sendCmd('SPD', num2str(val));
    triggerStep();
end

function onSpdEnter(~, event)
    if strcmp(event.Key, 'return'), onSetSpd(); end
end


% ======================== 动作按钮 ========================
function onRun(~,~),    sendCmd('RUN', '');  end
function onStop(~,~),   sendCmd('STOP', ''); end
function onRst(~,~),    sendCmd('RST', '');  end


% ======================== 清空曲线 ========================
function onClear(~, ~)
    global g_idx g_step
    g_idx = 0;
    g_step.n = 0;
    fprintf('曲线已清空\n');
end


% ======================== 编码器调试: 单轮测试 (新增) ========================
function onTestLeft(~, ~)
    sendCmd('ET', 'L');
    fprintf('左轮测试: 观察左上"实测轮速"绿线应为正值, 里程L递增\n');
end

function onTestRight(~, ~)
    sendCmd('ET', 'R');
    fprintf('右轮测试: 观察橙线应为正值, 里程R递增\n');
end

function onTestStop(~, ~)
    sendCmd('ET', 'S');
end


% ======================== 里程清零 (固件+轨迹原点) ========================
function onDistClear(~, ~)
    global g_traj
    sendCmd('DCLR', '');
    g_traj.x = 0;  g_traj.y = 0;  g_traj.n = 0;
    g_traj.xs = zeros(1, g_traj.max);
    g_traj.ys = zeros(1, g_traj.max);
    g_traj.prev_dL = 0;  g_traj.prev_dR = 0;   % 固件里程已归零
    fprintf('里程已清零, 轨迹原点已重置\n');
end


% ======================== 轨迹重置 (仅显示) ========================
function onTrajReset(~, ~)
    global g_traj
    g_traj.x = 0;  g_traj.y = 0;  g_traj.n = 0;
    g_traj.xs = zeros(1, g_traj.max);
    g_traj.ys = zeros(1, g_traj.max);
    g_traj.prev_dL = NaN;  g_traj.prev_dR = NaN;  % 下帧重新同步
    fprintf('轨迹已重置\n');
end


% ======================== 导出数据 (新增) ========================
function onExport(~, ~)
    global g_data g_idx g_traj
    if g_idx < 2
        fprintf('没有数据可导出\n');
        return;
    end
    [fname, fpath] = uiputfile('car_data.mat', '保存数据');
    if isequal(fname, 0), return; end
    logData = struct(...
        't', g_data.t(1:g_idx), 'yaw', g_data.yaw(1:g_idx), ...
        'err', g_data.err(1:g_idx), ...
        'pwmL', g_data.left(1:g_idx), 'pwmR', g_data.right(1:g_idx), ...
        'vL', g_data.vL(1:g_idx), 'vR', g_data.vR(1:g_idx), ...
        'dL', g_data.dL(1:g_idx), 'dR', g_data.dR(1:g_idx), ...
        'kp', g_data.kp(1:g_idx), 'ki', g_data.ki(1:g_idx), ...
        'kd', g_data.kd(1:g_idx), 'mode', g_data.mode(1:g_idx), ...
        'trajX', g_traj.xs(1:g_traj.n), 'trajY', g_traj.ys(1:g_traj.n));
    save(fullfile(fpath, fname), 'logData');
    fprintf('数据已导出: %s (%d 样本)\n', fullfile(fpath, fname), g_idx);
end


% ======================== 遥控: 原地转弯 (点击) ========================
function onTL90(~,~),  sendCmd('TL90', '');  end
function onTL180(~,~), sendCmd('TL180', ''); end
function onTR90(~,~),  sendCmd('TR90', '');  end
function onTR180(~,~), sendCmd('TR180', ''); end


% ======================== 遥控: 方向控制 (按住) ========================
function onFwdStart(~,~)
    global g_holdAction
    g_holdAction = 'FWD';
    sendCmd('FWD', '');
end

function onBwdStart(~,~)
    global g_holdAction
    g_holdAction = 'BWD';
    sendCmd('BWD', '');
end

function onCwStart(~,~)
    global g_holdAction
    g_holdAction = 'CW';
    sendCmd('CW', '');
end

function onCcwStart(~,~)
    global g_holdAction
    g_holdAction = 'CCW';
    sendCmd('CCW', '');
end

function onRcStop(~,~)
    global g_holdAction
    g_holdAction = '';
    sendCmd('REL', '');
end

function onMouseUp(~, ~)
    global g_holdAction
    if ~isempty(g_holdAction)
        sendCmd('REL', '');
        g_holdAction = '';
    end
end


% ======================== PID 自动整定: 启动/手动停止 ========================
function onAutoTune(~, ~)
    global g_tune g_idx g_step h_autotune_btn h_tunetext

    if g_tune.active
        % 再次点击 = 手动提前结束并分析
        fprintf('手动结束采集, 开始分析...\n');
        finishTune();
        return;
    end

    g_tune.active = true;
    g_tune.t0     = tic;
    g_tune.count  = 0;
    g_tune.halfcycles = 0;

    g_idx = 0;
    g_step.n = 0;

    set(h_autotune_btn, 'String', '停止并分析');
    set(h_tunetext, 'String', '自动整定: 采集中...');

    sendCmd('TUNE', 'ON');
    fprintf('\n=== PID 自动整定开始 ===\n');
    fprintf('继电反馈模式已启动 (±35), 采集中...\n');
    fprintf('收集到6个半周期(>=5s)自动分析, 20s超时, 也可再点按钮提前分析\n');
    fprintf('请确保小车在赛道上且能看到黑线!\n');
end


% ======================== 自动整定: 结束并分析 ========================
function finishTune()
    global g_tune h_autotune_btn h_tunetext

    if ~g_tune.active
        return;     % 已在分析/已结束, 防止定时器与按钮重复触发
    end
    g_tune.active = false;

    sendCmd('TUNE', 'OFF');
    set(h_autotune_btn, 'String', '自动整定');
    set(h_tunetext, 'String', '自动整定: 分析中...');

    n = g_tune.count;
    if n < 30
        fprintf('数据不足 (%d 点), 无法分析\n', n);
        set(h_tunetext, 'String', '整定失败: 数据不足');
        return;
    end

    err = g_tune.err_buf(1:n);
    t   = g_tune.t_buf(1:n);

    % 去掉前0.5s过渡过程
    keep = t >= (t(1) + 0.5);
    err = err(keep);
    t   = t(keep);
    if numel(err) < 20
        fprintf('过渡后数据不足\n');
        set(h_tunetext, 'String', '整定失败: 数据不足');
        return;
    end

    % 去均值 (消除偏置)
    err = err - mean(err);

    % ---- 滞回过零检测 (抗整数量化噪声) ----
    DEAD = 0.5;     % 死区阈值: |err|>0.5 才判定穿越
    crossings = [];
    state = 0;
    for i = 2:numel(err)
        if state <= 0 && err(i) > DEAD
            frac = (DEAD - err(i-1)) / (err(i) - err(i-1));
            crossings(end+1) = t(i-1) + frac * (t(i) - t(i-1)); %#ok<AGROW>
            state = 1;
        elseif state >= 0 && err(i) < -DEAD
            frac = (-DEAD - err(i-1)) / (err(i) - err(i-1));
            crossings(end+1) = t(i-1) + frac * (t(i) - t(i-1)); %#ok<AGROW>
            state = -1;
        end
    end

    if numel(crossings) < 4
        fprintf('振荡不明显 (有效穿越 %d 个 < 4), 无法整定\n', numel(crossings));
        set(h_tunetext, 'String', '整定失败: 振荡不明显');
        return;
    end

    % ---- 周期: 相邻穿越时间差=半周期, 用中位数抗离群 ----
    half_periods = diff(crossings);
    Tu = 2 * median(half_periods);

    % ---- 振幅: 每半周期内的峰值, 用中位数抗噪声尖峰 ----
    peaks = zeros(1, numel(crossings)-1);
    for k = 1:numel(crossings)-1
        seg = (t >= crossings(k) & t <= crossings(k+1));
        if any(seg)
            peaks(k) = max(abs(err(seg)));
        end
    end
    peaks = peaks(peaks > 0);
    if isempty(peaks)
        fprintf('无法测量振幅\n');
        set(h_tunetext, 'String', '整定失败: 振幅无效');
        return;
    end
    a = median(peaks);

    % ---- 振荡规则性校验 ----
    cvT = std(half_periods) / mean(half_periods);
    cvA = std(peaks) / mean(peaks);
    regular = (cvT < 0.35) && (cvA < 0.45);

    if a < 0.5
        fprintf('振幅过小 (%.2f), 未产生有效振荡\n', a);
        set(h_tunetext, 'String', '整定失败: 振幅过小');
        return;
    end

    % ---- 临界增益 (Åström-Hägglund) ----
    d  = 35.0;                      % 继电幅值 (必须与固件 RELAY_AMP 一致)
    Ku = 4 * d / (pi * a);

    fprintf('\n========================================\n');
    fprintf('  PID 自动整定结果 (继电反馈法)\n');
    fprintf('========================================\n');
    fprintf('  有效穿越数:    %d\n', numel(crossings));
    fprintf('  振荡周期 Tu:   %.3f s  (CV=%.2f)\n', Tu, cvT);
    fprintf('  振荡振幅  a:   %.2f    (CV=%.2f)\n', a, cvA);
    fprintf('  临界增益 Ku:   %.3f\n', Ku);
    fprintf('  振荡规则性:    %s\n', char(string(regular)));
    fprintf('========================================\n\n');

    if ~regular
        set(h_tunetext, 'String', ...
            sprintf('振荡不规则(CV_T=%.2f CV_a=%.2f), 结果仅供参考', cvT, cvA));
    else
        set(h_tunetext, 'String', ...
            sprintf('整定完成: Tu=%.2fs a=%.1f Ku=%.2f', Tu, a, Ku));
    end

    autoTuneDialog(Ku, Tu, a, cvT, cvA, regular, err, t, crossings);
end


% ======================== 自动整定结果对话框 (4套规则任选) ========================
function autoTuneDialog(Ku, Tu, a, cvT, cvA, regular, err, t, crossings)
    global h_kp h_ki h_kd s_serial g_step g_t0

    % 并联型整定规则 (固件: out=Kp*e + Ki*∫e + Kd*de/dt)
    %   规则以 Kp, Ti, Td 给出, 换算 Ki=Kp/Ti, Kd=Kp*Td
    rules = {
        '经典ZN(激进)',      0.60*Ku, 0.5*Tu,  0.125*Tu;
        '微超调',            0.33*Ku, 0.5*Tu,  Tu/3;
        '无超调(推荐)',      0.20*Ku, 0.5*Tu,  Tu/3;
        'Tyreus-Luyben',    0.45*Ku, 2.2*Tu,  Tu/6.3;
    };
    nRules = size(rules, 1);
    Kps = zeros(nRules,1); Kis = zeros(nRules,1); Kds = zeros(nRules,1);
    for i = 1:nRules
        Kps(i) = rules{i,2};
        Kis(i) = Kps(i) / rules{i,3};
        Kds(i) = Kps(i) * rules{i,4};
        % 与固件一致的限幅
        Kps(i) = min(max(Kps(i), 0), 3.0);
        Kis(i) = min(max(Kis(i), 0), 2.0);
        Kds(i) = min(max(Kds(i), 0), 0.02);
    end

    dlg = figure('Name', 'PID 自动整定结果', ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
        'Position', [480 200 460 560], 'Color', [0.95 0.95 0.97]);

    uicontrol(dlg, 'Style', 'text', 'String', '自动整定结果', ...
        'Position', [150 520 160 28], 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.95 0.95 0.97], 'ForegroundColor', [0.10 0.45 0.85]);

    if regular
        quality = '振荡规则性: 良好';
        qColor = [0.0 0.6 0.3];
    else
        quality = '振荡不规则, 结果仅供参考!';
        qColor = [0.8 0.2 0.2];
    end
    info = sprintf(['振荡周期 Tu = %.3f s   (CV=%.2f)\n' ...
                    '振荡振幅 a  = %.2f      (CV=%.2f)\n' ...
                    '临界增益 Ku = %.3f\n%s'], Tu, cvT, a, cvA, Ku, quality);
    uicontrol(dlg, 'Style', 'text', 'String', info, ...
        'Position', [30 455 400 60], 'FontSize', 10, ...
        'BackgroundColor', 'w', 'ForegroundColor', qColor);

    % 振荡波形 + 穿越点标记
    ax = axes(dlg, 'Position', [0.10 0.50 0.84 0.24]);
    plot(ax, t - t(1), err, '-', 'Color', [0.10 0.45 0.85], 'LineWidth', 1.0);
    hold(ax, 'on');
    for k = 1:numel(crossings)
        plot(ax, [crossings(k)-t(1) crossings(k)-t(1)], [-8 8], ':', ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 0.6);
    end
    plot(ax, [0 t(end)-t(1)], [0 0], '--', 'Color', [0.4 0.4 0.4]);
    title(ax, '极限环振荡波形 (竖线=滞回穿越点)', 'FontSize', 9);
    xlabel(ax, '时间 (s)');  ylabel(ax, '误差(去均值)');
    grid(ax, 'on');  ylim(ax, [-8 8]);

    % 4套规则 + 应用按钮
    uicontrol(dlg, 'Style', 'text', 'String', ...
        '选择整定规则 (点击即下发到小车):', ...
        'Position', [30 268 400 20], 'FontSize', 10, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.95 0.95 0.97], 'ForegroundColor', [0.12 0.12 0.12]);

    for i = 1:nRules
        yPos = 230 - (i-1) * 46;
        lbl = sprintf('%s\nKp=%.3f  Ki=%.4f  Kd=%.4f', ...
            rules{i,1}, Kps(i), Kis(i), Kds(i));
        uicontrol(dlg, 'Style', 'pushbutton', 'String', lbl, ...
            'Position', [40 yPos 250 40], 'FontSize', 9, ...
            'BackgroundColor', [0.18 0.60 0.28], 'ForegroundColor', 'w', ...
            'Callback', @(~,~)applyRule(i));
    end

    uicontrol(dlg, 'Style', 'pushbutton', 'String', '关闭', ...
        'Position', [320 100 100 120], 'FontSize', 12, ...
        'BackgroundColor', [0.50 0.50 0.53], 'ForegroundColor', 'w', ...
        'Callback', @(~,~)delete(dlg));

    function applyRule(idx)
        set(h_kp, 'String', sprintf('%.2f', Kps(idx)));
        set(h_ki, 'String', sprintf('%.3f', Kis(idx)));
        set(h_kd, 'String', sprintf('%.3f', Kds(idx)));
        write(s_serial, sprintf('$C,KP,%.3f\r\n', Kps(idx)), 'char');
        write(s_serial, sprintf('$C,KI,%.4f\r\n', Kis(idx)), 'char');
        write(s_serial, sprintf('$C,KD,%.4f\r\n', Kds(idx)), 'char');
        g_step.t0 = toc(g_t0);
        g_step.n = 0;
        fprintf('已应用 [%s]: Kp=%.3f Ki=%.4f Kd=%.4f\n', ...
            rules{idx,1}, Kps(idx), Kis(idx), Kds(idx));
        delete(dlg);
    end
end


% ======================== 定时器回调 ========================
function onTimer(~, ~, s, t0)
    global g_data g_idx g_running rx_buf ...
           h_err_line h_vL_line h_vR_line h_vAvg_line ...
           h_pidtext h_modetext h_yawtext h_errtext h_veltext h_enctext ...
           h_kp h_ki h_kd h_spd h_step_line h_mode g_step g_traj ...
           h_traj_line h_traj_dot h_traj_start g_tune h_tunetext

    if ~g_running, return; end

    % ---- 读取串口数据 ----
    if s.NumBytesAvailable > 0
        rx_buf = [rx_buf, read(s, s.NumBytesAvailable, 'char')];
    end

    % ---- 按行解析 (定位换行符, 不用strtok) ----
    while true
        nl = find(rx_buf == newline, 1);
        if isempty(nl), break; end
        lineStr = strtrim(rx_buf(1:nl-1));
        rx_buf  = rx_buf(nl+1:end);

        if strlength(lineStr) < 4 || ~startsWith(lineStr, '$T,')
            continue;
        end

        % 统一按数字解析全部字段 (mask为十进制)
        % 字段顺序: 1yaw 2err 3L% 4R% 5kp 6ki 7kd 8mode 9state 10mask
        %           11spd 12vL 13vR 14dL 15dR
        vals = sscanf(strrep(lineStr(4:end), ',', ' '), '%g', 15).';
        if numel(vals) < 11, continue; end
        if numel(vals) < 15
            vals(end+1:15) = 0;    % 旧固件帧兼容: 无编码器字段补0
        end

        % ---- 写入缓冲 (满则保留后半, 均摊O(1)) ----
        g_idx = g_idx + 1;
        if g_idx > numel(g_data.t)
            half = floor(numel(g_data.t) / 2);
            f = fieldnames(g_data);
            for k = 1:numel(f)
                v = g_data.(f{k});
                g_data.(f{k}) = [v(half+1:end), zeros(1, half)];
            end
            g_idx = half + 1;
        end

        g_data.t(g_idx)     = toc(t0);
        g_data.yaw(g_idx)   = vals(1);
        g_data.err(g_idx)   = vals(2);
        g_data.left(g_idx)  = vals(3);
        g_data.right(g_idx) = vals(4);
        g_data.kp(g_idx)    = vals(5);
        g_data.ki(g_idx)    = vals(6);
        g_data.kd(g_idx)    = vals(7);
        g_data.mode(g_idx)  = vals(8);
        g_data.state(g_idx) = vals(9);
        g_data.mask(g_idx)  = vals(10);
        g_data.speed(g_idx) = vals(11);
        g_data.vL(g_idx)    = vals(12);
        g_data.vR(g_idx)    = vals(13);
        g_data.dL(g_idx)    = vals(14);
        g_data.dR(g_idx)    = vals(15);

        % ---- 记录阶跃响应 (真实时间) ----
        if g_step.n < 500
            dt_step = g_data.t(g_idx) - g_step.t0;
            if dt_step >= 0
                g_step.n = g_step.n + 1;
                g_step.ts(g_step.n)   = dt_step;
                g_step.vals(g_step.n) = g_data.err(g_idx);
            end
        end

        % ---- 航位推算: 编码器里程增量 + 陀螺仪航向 ----
        dL = vals(14);  dR = vals(15);
        if isnan(g_traj.prev_dL)
            % 首帧: 只同步基准
            g_traj.prev_dL = dL;  g_traj.prev_dR = dR;
        else
            ddL = dL - g_traj.prev_dL;
            ddR = dR - g_traj.prev_dR;
            g_traj.prev_dL = dL;  g_traj.prev_dR = dR;
            if abs(ddL) > 50 || abs(ddR) > 50
                % 里程被清零(跳变): 跳过本帧增量, 不移动轨迹点
            else
                ds = (ddL + ddR) / 2;          % 平均速度走过的弧长(cm)
                yaw_rad = deg2rad(g_data.yaw(g_idx));
                g_traj.x = g_traj.x + ds * cos(yaw_rad);
                g_traj.y = g_traj.y + ds * sin(yaw_rad);
                g_traj.n = g_traj.n + 1;
                if g_traj.n > g_traj.max
                    half = floor(g_traj.max / 2);
                    g_traj.xs = [g_traj.xs(half+1:end), zeros(1, half)];
                    g_traj.ys = [g_traj.ys(half+1:end), zeros(1, half)];
                    g_traj.n = half + 1;
                end
                g_traj.xs(g_traj.n) = g_traj.x;
                g_traj.ys(g_traj.n) = g_traj.y;
            end
        end

        % ---- 自动整定: 采集数据 ----
        if g_tune.active
            if g_tune.count < 4000
                g_tune.count = g_tune.count + 1;
                g_tune.err_buf(g_tune.count) = g_data.err(g_idx);
                g_tune.t_buf(g_tune.count)   = g_data.t(g_idx);
            end
        end

        % ---- 更新顶部状态栏 ----
        modeNames = {'TRACK','A->B','ABCD','ACBD','AC4','TRK50','REMOTE'};
        modeIdx = round(g_data.mode(g_idx)) + 1;
        if modeIdx >= 1 && modeIdx <= 7
            modeStr = modeNames{modeIdx};
        else
            modeStr = '?';
        end
        set(h_pidtext, 'String', sprintf('Kp=%.3f  Ki=%.4f  Kd=%.4f  SPD=%d', ...
            g_data.kp(g_idx), g_data.ki(g_idx), g_data.kd(g_idx), ...
            round(g_data.speed(g_idx))));
        set(h_modetext, 'String', sprintf('%s MASK=%02X', modeStr, ...
            round(g_data.mask(g_idx))));
        set(h_yawtext, 'String', sprintf('Yaw=%+.1f', g_data.yaw(g_idx)));
        vAvg = (g_data.vL(g_idx) + g_data.vR(g_idx)) / 2;
        set(h_veltext, 'String', sprintf('V: L%+.1f R%+.1f AV%+.1f cm/s', ...
            g_data.vL(g_idx), g_data.vR(g_idx), vAvg));
        set(h_enctext, 'String', sprintf('D: L=%.1f R=%.1f cm', ...
            g_data.dL(g_idx), g_data.dR(g_idx)));
        set(h_errtext, 'String', sprintf('Err=%+d', round(g_data.err(g_idx))));

        % ---- 同步模式下拉框 ----
        if get(h_mode, 'Value') ~= modeIdx
            set(h_mode, 'Value', modeIdx);
        end

        % ---- 首次同步输入框 ----
        if g_idx == 1
            set(h_kp, 'String', sprintf('%.2f', g_data.kp(g_idx)));
            set(h_ki, 'String', sprintf('%.3f', g_data.ki(g_idx)));
            set(h_kd, 'String', sprintf('%.3f', g_data.kd(g_idx)));
            set(h_spd, 'String', num2str(round(g_data.speed(g_idx))));
        end
    end

    % ---- 自动整定: 检查停止条件 ----
    if g_tune.active
        elapsed = toc(g_tune.t0);
        % 增量统计半周期数 (死区0.5)
        if g_tune.count >= 10
            e = g_tune.err_buf(1:g_tune.count);
            e = e - mean(e);
            up = nnz(e(1:end-1) <= 0.5 & e(2:end) > 0.5);
            dn = nnz(e(1:end-1) >= -0.5 & e(2:end) < -0.5);
            g_tune.halfcycles = up + dn;
        end
        set(h_tunetext, 'String', sprintf('整定采集: %.0fs 半周期=%d (6个自动分析)', ...
            elapsed, g_tune.halfcycles));
        if (g_tune.halfcycles >= 6 && elapsed >= 5) || elapsed >= 20
            finishTune();
        end
    end

    % ---- 更新图表 ----
    if g_idx > 1
        t = g_data.t(1:g_idx);
        tMax = t(end);
        tMin = max(0, tMax - 10);

        set(h_err_line, 'XData', t, 'YData', g_data.err(1:g_idx));
        ax1 = ancestor(h_err_line, 'axes');
        xlim(ax1, [tMin tMax]);

        set(h_vL_line, 'XData', t, 'YData', g_data.vL(1:g_idx));
        set(h_vR_line, 'XData', t, 'YData', g_data.vR(1:g_idx));
        set(h_vAvg_line, 'XData', t, 'YData', (g_data.vL(1:g_idx) + g_data.vR(1:g_idx)) / 2);
        ax2 = ancestor(h_vL_line, 'axes');
        xlim(ax2, [tMin tMax]);

        if g_step.n > 1
            set(h_step_line, 'XData', g_step.ts(1:g_step.n), ...
                             'YData', g_step.vals(1:g_step.n));
        end

        if g_traj.n > 1
            set(h_traj_line, 'XData', g_traj.xs(1:g_traj.n), ...
                             'YData', g_traj.ys(1:g_traj.n));
            set(h_traj_dot, 'XData', g_traj.x, 'YData', g_traj.y);
            set(h_traj_start, 'XData', g_traj.xs(1), 'YData', g_traj.ys(1));
            ax4 = ancestor(h_traj_line, 'axes');
            xmin = min(g_traj.xs(1:g_traj.n));  xmax = max(g_traj.xs(1:g_traj.n));
            ymin = min(g_traj.ys(1:g_traj.n));  ymax = max(g_traj.ys(1:g_traj.n));
            margin = max(20, (max(xmax-xmin, ymax-ymin)) * 0.15);
            xlim(ax4, [xmin-margin, xmax+margin]);
            ylim(ax4, [ymin-margin, ymax+margin]);
        end

        drawnow limitrate;
    end
end


% ======================== 关闭窗口 ========================
function onClose(~, ~)
    global g_running g_timer s_serial
    g_running = false;
    try
        if isvalid(g_timer)
            stop(g_timer);
            delete(g_timer);
        end
    catch
    end
    try
        delete(s_serial);   % serialport对象: delete即关闭
    catch
    end
    delete(gcf);
    fprintf('\n程序已退出, 串口已关闭。\n');
end
