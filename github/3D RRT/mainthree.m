clear all;
close all;
clc;
tic

%% 定义变量
axisStart = [0 0 0];
axisLWH = [200 200 200];
it = 0;
itmax=100;
% 定义障碍物
cubeInfo.exist = 0;
cylinderInfo.exist = 0;
sphereInfo.exist = 0;

pathPoint = [0 0 0;
%             100 50 110;
%             50 100 110;
             200 200 200];  % 一系列的路径点
 
cubeInfo = createCubeObject(cubeInfo);       % 创建长方体障碍物信息
cylinderInfo = createCylinderObject(cylinderInfo);  % 创建圆柱障碍物信息
sphereInfo = createSphereObject(sphereInfo);    % 创建球形障碍物信息
 
%% 画图
fig = figure;
lineHandles = []; % 存储绘制的线句柄
treeHandles = []; % 存储RRT树的句柄
bezierHandle = [];% 存放贝塞尔曲线的句柄
colorMatCube = [1 0.3 0.2];
colorMatCylinder = [0 1 0];
colorMatSphere = [0 0 1];
pellucidity = 0.6;    % 透明度
hold on;
scatter3(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
scatter3(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
drawCubeObject(cubeInfo, colorMatCube, pellucidity);     % 画长方体障碍物
drawCylinderObject(cylinderInfo, colorMatCylinder, pellucidity);   % 画圆柱体障碍物
drawSphereObject(sphereInfo, colorMatSphere, pellucidity);  % 画球形障碍物
text(pathPoint(1,1), pathPoint(1,2), pathPoint(1,3), '起点');
text(pathPoint(end,1), pathPoint(end,2), pathPoint(end,3), '终点');
% view(3)
% grid on;
axis equal;
axis([0 200 0 200 0 200])
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
 
%% 寻找路径
while it < itmax
    it = it + 1;
    totalPath = [];
    
    % 删除之前RRT树的绘制
    if ~isempty(treeHandles)
        delete(treeHandles);
    end
    treeHandles = [];
    
    % RRT路径生成
    for k1 = 1:size(pathPoint, 1)-1
        startPoint = pathPoint(k1,:);
        goalPoint = pathPoint(k1+1,:);

        [Path, newTreeHandles] = RRT(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
        if ~isempty(Path)  % 判断是否为空数组，若不是，继续执行
            treeHandles = [treeHandles; newTreeHandles]; % 保存RRT树的句柄
            for k2 = 1:size(Path,1)-1
                h = line([Path(k2,1) Path(k2+1,1)], [Path(k2,2) Path(k2+1,2)], [Path(k2,3) Path(k2+1,3)], 'LineWidth', 1, 'Color', 'r');
                treeHandles = [treeHandles; h]; % 保存路径线段的句柄
                lineHandles = [lineHandles; h]; % 保存每段线的句柄
            end
            totalPath = [totalPath; Path];   % 拼接起来，刚开始为空
        end
    end

    %% 检查路径点数量
    if size(totalPath, 1) < 4
        disp('路径点数量不足，继续寻找路径');
        delete(lineHandles);
        continue;
    end

    %% 分段三阶贝塞尔曲线拟合(未进行导数连续)
%     re = [];
%     for k = 1:3:size(totalPath, 1)-1
%         if k + 3 <= size(totalPath, 1)
%             P0 = totalPath(k, :);
%             P1 = totalPath(k+1, :);
%             P2 = totalPath(k+2, :);
%             P3 = totalPath(k+3, :);
%             t = linspace(0, 1, 100)';
%             B = (1-t).^3 .* P0 + 3*(1-t).^2 .* t .* P1 + 3*(1-t) .* t.^2 .* P2 + t.^3 .* P3;
%             re = [re; B];
%         end
%     end
%     bezierHandle = plot3(re(:,1), re(:,2), re(:,3), 'k', 'LineWidth', 1.5);
%     hold on;
    %% 分段三阶贝塞尔曲线拟合(进行导数连续)
re = [];
for k = 1:3:size(totalPath, 1)-1
    if k + 3 <= size(totalPath, 1)
        P0 = totalPath(k, :);
        P1 = totalPath(k+1, :);
        P2 = totalPath(k+2, :);
        P3 = totalPath(k+3, :);
        t = linspace(0, 1, 100)';
        B = (1-t).^3 .* P0 + 3*(1-t).^2 .* t .* P1 + 3*(1-t) .* t.^2 .* P2 + t.^3 .* P3;
        if ~isempty(re)
            % Remove the first point to avoid duplication
            B = B(2:end, :);
        end
        re = [re; B];
        % Calculate the derivative at the end of this segment
        if k ~= 1
            dP0 = 3 * (P1 - P0);
            % Use the derivative as the initial derivative for the next segment
            P1 = P0 + dP0;
        end
    end
end

bezierHandle = plot3(re(:,1), re(:,2), re(:,3), 'b', 'LineWidth', 1.5);
    %% 判断贝塞尔曲线是否发生碰撞

    cubeFlagB = isCubeB(cubeInfo, length(re), re);   % 长方体碰撞检测函数
    cylinderFlagB = isCylinderB(cylinderInfo, length(re), re);  % 圆柱体碰撞检测函数
    sphereFlagB = isSphereB(sphereInfo, length(re), re);   % 球形障碍物碰撞检测函数
    
    if cubeFlagB || cylinderFlagB || sphereFlagB % 或，有一个为1，说明发生碰撞，执行if中的continue，返回while下
        disp('检测到碰撞，进行下一次循环');
        disp(['这是第', num2str(it), '次循环。']);
        delete(lineHandles);
        delete(bezierHandle);
        delete(treeHandles);
        continue;
    end
    disp('找到无碰撞路径，跳出循环');
    break;
end

if it == itmax
    disp('路径规划失败');
    return;
end

%% 实际路径长度
% T = totalPath;
% sum = 0;
% for n = 1:size(T, 1)-1
%     sum = sum + norm(T(n+1, :) - T(n, :));
% end
% disp(['优化前路径长度=', num2str(sum)]);
% 
% %% 优化后路径长度
% sun = 0;
% for n = 1:size(re, 1)-1
%     sun = sun + norm(re(n+1, :) - re(n, :));
% end
% disp(['优化后路径长度=', num2str(sun)]);
toc
