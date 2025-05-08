function [Path, treeHandles] = RRTconnect(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% RRT算法寻找路径点

%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 5000000;   %树扩展最大迭代次数
iter = 0;   %当前迭代次数
step = 5;  %步长
count = 1;  %计数器
Thr = 10;   %阈值
Path = [];
treeHandles = []; 

%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.z(1) = startPoint(3);
T.pre(1) = 0;

while iter < iterMax
    iter = iter + 1;
    
    %% 在空间中随机采样
    randCoor = samplePoint(axisStart, axisLWH, goalPoint);%newcoor=Xrand
    
    %% 寻找树上最近点
    [nearCoor, preIndex] = findNearPoint(randCoor, T);
    
    %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor, randCoor, step);
    
    %% 碰撞检测
    cubeFlag = isCubeCollision(cubeInfo, nearCoor, newCoor, step);   %长方体碰撞检测函数
    cylinderFlag = isCylinderCollision(cylinderInfo, nearCoor, newCoor, step);  %圆柱体碰撞检测函数
    sphereFlag = isSphereCollision(sphereInfo, nearCoor, newCoor, step);   %球形障碍物碰撞检测函数
    
    if cubeFlag || cylinderFlag || sphereFlag    %或，有一个为1，说明发生碰撞，执行if中的continue，返回while下,开始下一循环，
        % continue语句跳过for或while循环中剩余的说明，并开始下一迭代。要完全退出循环，请使用break语句。
        continue;
    end
    
    %% 将新点插入树中
    count = count + 1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.z(count) = newCoor(3);
    T.pre(count) = preIndex;
    
    % 绘制每一个新点并保存句柄
    h = line([nearCoor(1) newCoor(1)], [nearCoor(2) newCoor(2)], [nearCoor(3) newCoor(3)], 'LineWidth', 1); 
    treeHandles = [treeHandles; h]; % 保存句柄
    pause(0.01);
    
    if calcuDis(newCoor, goalPoint) < Thr
        break; % 跳出while循环
    end 
end

if iter == iterMax
    Path = [];
    disp('路径规划失败');
    return;
end

%% 寻找路径
index = T.pre(end);
count = 1;

while T.pre(index) ~= 0
    Path(count, 1) = T.x(index);
    Path(count, 2) = T.y(index);
    Path(count, 3) = T.z(index);
    index = T.pre(index);
    count = count + 1;
end

%将初始点添加到Path中
Path(count, 1) = startPoint(1);
Path(count, 2) = startPoint(2);
Path(count, 3) = startPoint(3);

%将目标点添加到Path中,起点到终点绘图
Path = flipud(Path);%翻转
count = count + 1;
Path(count, 1) = goalPoint(1);
Path(count, 2) = goalPoint(2);
Path(count, 3) = goalPoint(3);

end
