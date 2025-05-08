% function [Path, treeHandles] = RRT_connect(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
% %% RRTConnect算法寻找路径点
% 
% %% 变量定义
% calcuDis = @(x,y) sqrt((x(1)-y(1))^2 + (x(2)-y(2))^2 + (x(3)-y(3))^2); % 距离计算函数
% iterMax = 500000; % 最大迭代次数
% iter = 0; % 当前迭代次数
% step = 5; % 步长
% Thr = 10; % 阈值
% Path = [];
% 
% 
% % 构建树A和树B
% treeA.x = startPoint(1);
% treeA.y = startPoint(2);
% treeA.z = startPoint(3);
% % treeA.pre = 0;
% 
% treeB.x = goalPoint(1);
% treeB.y = goalPoint(2);
% treeB.z = goalPoint(3);
% % treeB.pre = 0;
% 
% flag = false;
% 
% while iter < iterMax
%     iter = iter + 1;
%     
%     %% 在空间中随机采样
%     randCoor = samplePoint(axisStart, axisLWH, goalPoint);
%     
%     %% 树A向随机点扩展
%     [treeA, newCoorA, preIndexA, successA] = extendTree(treeA, randCoor, step, Thr, cubeInfo, cylinderInfo, sphereInfo);
%     if successA
%         hA = line([treeA.x(preIndexA) newCoorA(1)], [treeA.y(preIndexA) newCoorA(2)], [treeA.z(preIndexA) newCoorA(3)], 'LineWidth', 1, 'Color', 'r');
%      
%         pause(0.01);
%         
%         %% 树B向新点扩展
%         [treeB, newCoorB, preIndexB, successB] = extendTree(treeB, newCoorA, step, Thr, cubeInfo, cylinderInfo, sphereInfo);
%         if successB
%             hB = line([treeB.x(preIndexB) newCoorB(1)], [treeB.y(preIndexB) newCoorB(2)], [treeB.z(preIndexB) newCoorB(3)], 'LineWidth', 1, 'Color', 'b');
%          
%             pause(0.01);
%             
%             %% 检查两棵树是否连接
%             if calcuDis(newCoorA, newCoorB) < Thr
%                 flag = true;
%                 break;
%             end
%         end
%     end
%     
%     %% 交换树A和树B
%     [treeA, treeB] = deal(treeB, treeA);
%      %% 碰撞检测
%     cubeFlag = isCubeCollision(cubeInfo, nearCoor, newCoor, step); % 长方体碰撞检测函数
%     cylinderFlag = isCylinderCollision(cylinderInfo, nearCoor, newCoor, step); % 圆柱体碰撞检测函数
%     sphereFlag = isSphereCollision(sphereInfo, nearCoor, newCoor, step); % 球形障碍物碰撞检测函数
%     
%     if cubeFlag || cylinderFlag || sphereFlag % 如果发生碰撞，退出循环
%         continue;
%     end
%     
%     % 将新点插入树中
%     tree.x(end+1) = newCoor(1);
%     tree.y(end+1) = newCoor(2);
%     tree.z(end+1) = newCoor(3);
%     tree.pre(end+1) = preIndex;
%     preIndex = length(tree.x);
%     
%     % 检查是否接近目标
%     if calcuDis(newCoor, randCoor) < Thr
%         success = true;
%         break;
%     end
% end
% 
% if ~flag
%     Path = [];
%     disp('路径规划失败');
%     return;
% end
% 
% %% 寻找路径
% PathA = findPath(treeA, newCoorA);
% PathB = findPath(treeB, newCoorB);
% 
% % 合并路径
% Path = [flipud(PathA); PathB];
% 
% end
function [Path, treeHandles] = RRT_connect(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
    %% RRTConnect算法寻找路径点

    %% 变量定义
    calcuDis = @(x,y) sqrt((x(1)-y(1))^2 + (x(2)-y(2))^2 + (x(3)-y(3))^2); % 距离计算函数
    iterMax = 500000; % 最大迭代次数
    iter = 0; % 当前迭代次数
    step = 5; % 步长
    Thr = 10; % 阈值
    Path = [];
    treeHandles = []; % 初始化 treeHandles
    % 构建树A和树B
    treeA = struct('x', [], 'y', [], 'z', [], 'pre', []);
    treeA.x = [startPoint(1)];
    treeA.y = [startPoint(2)];
    treeA.z = [startPoint(3)];
    treeA.pre = [0];

    treeB = struct('x', [], 'y', [], 'z', [], 'pre', []);
    treeB.x = [goalPoint(1)];
    treeB.y = [goalPoint(2)];
    treeB.z = [goalPoint(3)];
    treeB.pre = [0];

    flag = false;

    while iter < iterMax
        iter = iter + 1;

        %% 在空间中随机采样
        randCoor = samplePoint(axisStart, axisLWH, goalPoint);

        %% 树A向随机点扩展
        [treeA, newCoorA, preIndexA, successA] = extendTree(treeA, randCoor, step, Thr, cubeInfo, cylinderInfo, sphereInfo);
        if successA
            hA = line([treeA.x(preIndexA) newCoorA(1)], [treeA.y(preIndexA) newCoorA(2)], [treeA.z(preIndexA) newCoorA(3)], 'LineWidth', 1);
            treeHandles = [treeHandles; hA]; % 添加到 treeHandles
            pause(0.01);

            %% 树B向新点扩展
            [treeB, newCoorB, preIndexB, successB] = extendTree(treeB, newCoorA, step, Thr, cubeInfo, cylinderInfo, sphereInfo);
            if successB
                hB = line([treeB.x(preIndexB) newCoorB(1)], [treeB.y(preIndexB) newCoorB(2)], [treeB.z(preIndexB) newCoorB(3)], 'LineWidth', 1);
                treeHandles = [treeHandles; hB]; % 添加到 treeHandles
                pause(0.01);

                %% 检查两棵树是否连接
                if calcuDis(newCoorA, newCoorB) < Thr
                    flag = true;
                    break;
                end
            end
        end

        %% 交换树A和树B
        [treeA, treeB] = deal(treeB, treeA);
    end

    if ~flag
        Path = [];
        disp('路径规划失败');
        return;
    end

    %% 寻找路径
    PathA = findPath(treeA, newCoorA);
    PathB = findPath(treeB, newCoorB);

    % 合并路径
    Path = [flipud(PathA); PathB];
end
