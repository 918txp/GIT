function [Path, treeHandles] = RRTstar(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% RRT算法寻找路径点
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 500000000;   %树扩展最大迭代次数
iter = 0;   %当前迭代次数
e=2;
stepMax=20;
step = 5;  %步长
count = 1;  %计数器
Thr = 10;   %阈值
r=4*step;  %影响半径，大一些路径规划效果好，迭代慢，越小越接近RRT
flag=0;
treeHandles = []; 
w=0.1+0.4.*rand(1,1);
d0=step*w;
%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.z(1) = startPoint(3);
T.pre(1) = 0;
T.cost(1) = 0;
Path = [];
while iter < iterMax
    iter = iter + 1;
    
    %% 在空间中随机采样
    randCoor = samplePoint(axisStart, axisLWH, goalPoint);
    
    %% 寻找树上最近点
   [nearCoor,parentIndex] = FindNearstPoint(T,randCoor); 
    
    %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor, randCoor, step);
    %% 重写
%     parentIndex = preIndex;  % 初始化父节点为最近点的索引
    parentIndex = RewriteFunction(T,newCoor,r,parentIndex);
    %% 
     A = [T.x(parentIndex),T.y(parentIndex),T.z(parentIndex)];
     B = newCoor;
    collisionFlag = CollisionDetection(cubeInfo,cylinderInfo,sphereInfo,A,B,step);    
    if collisionFlag
%         step=d0;
        continue;
%     else
%         count=count+1;
%         step=min(e*step,stepMax);
    end   

    %% 碰撞检测
%     cubeFlag = isCubeCollision(cubeInfo, nearCoor, newCoor, step);   %长方体碰撞检测函数
%     cylinderFlag = isCylinderCollision(cylinderInfo, nearCoor, newCoor, step);  %圆柱体碰撞检测函数
%     sphereFlag = isSphereCollision(sphereInfo, nearCoor, newCoor, step);   %球形障碍物碰撞检测函数
%     
%     if cubeFlag || cylinderFlag || sphereFlag    %或，有一个为1，说明发生碰撞，执行if中的continue，返回while下
%         step=d0;
%         continue;
%     else
%     count = count + 1;
%     step=min(e*step,stepMax);
%     end
    %% 将新点插入树中
    count = count + 1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.z(count) = newCoor(3);
    T.pre(count) = parentIndex;
    T.cost(count) = CalcuDistance(A, B)+T.cost(parentIndex);
    line([A(1),B(1)],[A(2),B(2)],[A(3),B(3)],'LineWidth',1);   % 要是想只画最终的路径图，不画扩展图就把这行注释掉
    pause(0.01)  %不想看动画，就把这行注释        
    %绘制每一个新点并保存句柄
    h = line([nearCoor(1) newCoor(1)], [nearCoor(2) newCoor(2)], [nearCoor(3) newCoor(3)], 'LineWidth', 1); 
    treeHandles = [treeHandles; h]; %保存句柄
%     pause(0.01);
    %% 随机重连
    T=RandRelink(T,newCoor,cubeInfo,cylinderInfo,sphereInfo,step,r);
     if CalcuDistance(newCoor,goalPoint)<Thr
        flag = 1;
        break;
    end    
end
%% 路径规划失败直接返回
if ~flag 
    disp('路径规划失败');
    return;
else
    disp('路径规划成功');
end


% 
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
Path = flipud(Path);
count = count + 1;
Path(count, 1) = goalPoint(1);
Path(count, 2) = goalPoint(2);
Path(count, 3) = goalPoint(3);

end
