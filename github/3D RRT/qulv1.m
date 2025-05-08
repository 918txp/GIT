% 示例三维数据点
x = totalPath(:,1);
y = totalPath(:,2);
z = totalPath(:,3);

% 计算相邻点的差分
dx = diff(x);
dy = diff(y);
dz = diff(z);

% 计算相邻点间的距离
ds = sqrt(dx.^2 + dy.^2 + dz.^2);

% 计算一阶导数
dxds = dx ./ ds;
dyds = dy ./ ds;
dzds = dz ./ ds;

% 计算二次差分（导数的变化）
d2xds2 = diff(dxds);
d2yds2 = diff(dyds);
d2zds2 = diff(dzds);

% 评估导数变化的平稳性
smoothness_measure_x = std(d2xds2); % X 方向平稳性指标
smoothness_measure_y = std(d2yds2); % Y 方向平稳性指标
smoothness_measure_z = std(d2zds2); % Z 方向平稳性指标

% 综合平稳性指标
smoothness_measure = sqrt(smoothness_measure_x^2 + smoothness_measure_y^2 + smoothness_measure_z^2);

% 输出结果
disp(['X 方向二次差分的标准差：', num2str(smoothness_measure_x)]);
disp(['Y 方向二次差分的标准差：', num2str(smoothness_measure_y)]);
disp(['Z 方向二次差分的标准差：', num2str(smoothness_measure_z)]);
disp(['综合平稳性指标：', num2str(smoothness_measure)]);

% 画图展示结果
figure;

% % % 第一个子图：一阶导数
subplot(2, 1, 1);
plot(1:length(dxds), dxds, 1:length(dyds), dyds,  1:length(dzds), dzds,'LineWidth',2);
xlabel('Nodes', 'FontSize', 18, 'FontWeight', 'bold');      % 添加x轴标签
ylabel('First derivative', 'FontSize', 18, 'FontWeight', 'bold');    % 添加y轴标签
xlim([1 16]);ylim([-1 2]);
legend('dx/ds', 'dy/ds', 'dz/ds','orientation','horizontal','FontName','Times New Roman','FontSize',14);
legend boxoff; % Optional: Remove the legend box

ax = gca; % 获取当前坐标轴
ax.XAxis.FontSize = 14;  % X轴刻度文字单独调大
ax.YAxis.FontSize = 14;
ax.FontWeight = 'bold'; % 刻度加粗


% % % 第二个子图：二阶导数
subplot(2, 1, 2);
plot(1:length(d2xds2), d2xds2,  1:length(d2yds2), d2yds2,  1:length(d2zds2), d2zds2,'LineWidth',2);
xlabel('Nodes', 'FontSize', 18, 'FontWeight', 'bold');      % 添加x轴标签
ylabel('Second derivative', 'FontSize', 18, 'FontWeight', 'bold');    % 添加y轴标签
xlim([1 15]);ylim([-1 2]);
legend('d^2x/ds^2', 'd^2y/ds^2', 'd^2z/ds^2','orientation','horizontal','FontName','Times New Roman','FontSize',14);
legend boxoff; % Optional: Remove the legend box

ax = gca;
ax.XAxis.FontSize = 14;  % X轴刻度文字单独调大
ax.YAxis.FontSize = 14;
ax.FontWeight = 'bold';

% 判断曲线的平滑性
threshold = 0.1; % 根据具体应用设置阈值
if smoothness_measure < threshold
    disp('曲线较为平滑');
else
    disp('曲线不平滑');
end
%% 
%% 








% 
% % 示例三维数据点
% x = re(:,1);
% y = re(:,2);
% z = re(:,3);
% 
% % % % 计算相邻点的差分
% dx = diff(x);
% dy = diff(y);
% dz = diff(z);
% 
% % % % 计算相邻点间的距离
% ds = sqrt(dx.^2 + dy.^2 + dz.^2);
% 
% % % % 计算一阶导数
% dxds = dx ./ ds;
% dyds = dy ./ ds;
% dzds = dz ./ ds;
% 
% % % 计算二次差分（导数的变化）
% d2xds2 = diff(dxds);
% d2yds2 = diff(dyds);
% d2zds2 = diff(dzds);
% 
% % % 评估导数变化的平稳性
% smoothness_measure_x = std(d2xds2); % X 方向平稳性指标
% smoothness_measure_y = std(d2yds2); % Y 方向平稳性指标
% smoothness_measure_z = std(d2zds2); % Z 方向平稳性指标
% 
% % % % 综合平稳性指标
% smoothness_measure = sqrt(smoothness_measure_x^2 + smoothness_measure_y^2 + smoothness_measure_z^2);
% 
% % % 输出结果
% disp(['X 方向二次差分的标准差：', num2str(smoothness_measure_x)]);
% disp(['Y 方向二次差分的标准差：', num2str(smoothness_measure_y)]);
% disp(['Z 方向二次差分的标准差：', num2str(smoothness_measure_z)]);
% disp(['综合平稳性指标：', num2str(smoothness_measure)]);
% % 
% 
% % % 画图展示结果
% % % 画图展示结果
% figure;
% 
% % 第一个子图：一阶导数
% subplot(2, 1, 1);
% plot(1:length(dxds), dxds, 1:length(dyds), dyds,  1:length(dzds), dzds,'LineWidth',2);
% xlabel('Nodes', 'FontSize', 18, 'FontWeight', 'bold');      % 添加x轴标签
% ylabel('First derivative', 'FontSize', 18, 'FontWeight', 'bold');    % 添加y轴标签
% xlim([1 15]);ylim([-1 2]);
% legend('dx/ds', 'dy/ds', 'dz/ds','orientation','horizontal','FontName','Times New Roman','FontSize',14);
% legend boxoff; % Optional: Remove the legend box
% 
% ax = gca; % 获取当前坐标轴
% ax.XAxis.FontSize = 14;  % X轴刻度文字单独调大
% ax.YAxis.FontSize = 14;
% ax.FontWeight = 'bold'; % 刻度加粗
% 
% 
% % 第二个子图：二阶导数
% subplot(2, 1, 2);
% plot(1:length(d2xds2), d2xds2,  1:length(d2yds2), d2yds2,  1:length(d2zds2), d2zds2,'LineWidth',2);
% xlabel('Nodes', 'FontSize', 18, 'FontWeight', 'bold');      % 添加x轴标签
% ylabel('Second derivative', 'FontSize', 18, 'FontWeight', 'bold');    % 添加y轴标签
% xlim([1 14]);ylim([-1 2]);
% legend('d^2x/ds^2', 'd^2y/ds^2', 'd^2z/ds^2','orientation','horizontal','FontName','Times New Roman','FontSize',14);
% legend boxoff; % Optional: Remove the legend box
% 
% ax = gca;
% ax.XAxis.FontSize = 14;  % X轴刻度文字单独调大
% ax.YAxis.FontSize = 14;
% ax.FontWeight = 'bold';
% 
% 
% % % 判断曲线的平滑性
% threshold = 0.1; % 根据具体应用设置阈值
% if smoothness_measure < threshold
%     disp('曲线较为平滑');
% else
%     disp('曲线不平滑');
% end

