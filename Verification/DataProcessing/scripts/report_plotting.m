script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
data = readmatrix(fullfile(data_dir, '10ms', 'Source', 'imu_EM_log.csv'));

% 提取offset列（第三列）
offset = data(:,3); 

% 计算纵轴（转换为毫秒）
y = offset / 28000;  % 根据要求转换为ms

% 生成横轴（数据点序号）
x = 1:length(offset);

% 绘制散点图
figure
h = scatter(x, y, 40, 'o', ...          % 空心圆圈，大小60
    'MarkerEdgeColor', [0.2 0.6 0.8], ... % 设置边缘颜色
    'LineWidth', 0.3);                   % 边缘线宽

% 添加标签和标题
xlabel('Data Point Index', 'FontSize', 16)
ylabel('T_{OFFSET} (ms)', 'FontSize', 16)
title('T_{OFFSET} in Received Metadata Packages', 'FontSize', 20)

% 设置坐标轴范围
xlim([0 length(offset)+1])
ylim([0 max(y)*1.1])

% 添加网格线
grid on
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3)

% 添加图例
legend(h, 'Normalized Timing Offset', ...
    'Location', 'northwest', ...
    'FontSize', 10)

% 设置专业字体
set(gca, 'FontName', 'Arial', 'FontSize', 11)
set(findall(gcf, 'Type', 'text'), 'FontName', 'Arial')

% 优化图形显示
box on
