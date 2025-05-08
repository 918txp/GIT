%% 清空变量
clear all;
close all;
clc;
tic
%% 定义变量

axisStart = [0 0 0];
axisLWH = [200 200 100];
It = 0;     %针对需要优化路径
Itmax=100;
%定义障碍物
cubeInfo.exist = 0;
cylinderInfo.exist = 0;
sphereInfo.exist = 0;

% pathPoint = [80 0 50;
%             50 100 80
%             150 200 80];
pathPoint = [0 100 10;
            200 100 10];
%                0 0 0;
%              200 200 200];  %一系列的路径点
 
cubeInfo = createCubeObject(cubeInfo);       %创建长方体障碍物信息
cylinderInfo = createCylinderObject(cylinderInfo);  %创建圆柱障碍物信息
sphereInfo = createSphereObject(sphereInfo);    %创建球形障碍物信息
 
%% 画图
fig=figure ;
lineHandles = []; % 存储绘制的线句柄
treeHandles = []; % 存储RRT树的句柄
bezierHandle = [];%存放贝塞尔曲线的句柄
colorMatCube = [0.87 0.87 0.8];
colorMatCylinder = [0 1 0];
colorMatSphere = [0 0 1];
pellucidity = 1;    %透明度
hold on;
scatter3(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
scatter3(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),'MarkerEdgeColor','k','MarkerFaceColor','k');
drawCubeObject(cubeInfo,colorMatCube,pellucidity);     %画长方体障碍物
drawCylinderObject(cylinderInfo,colorMatCylinder,pellucidity);   %画圆柱体障碍物
drawSphereObject(sphereInfo,colorMatSphere,pellucidity);  %画球形障碍物
text(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'  start');
text(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),' goal');
% view(3)
% grid on;
axis equal;
axis([0 200 0 200 0 100])
xlabel('X(mm)','FontWeight', 'bold')
ylabel('Y(mm)','FontWeight', 'bold')
zlabel('Z(mm)','FontWeight', 'bold')
set(gca,'FontName','Times New Roman','FontSize',11,'FontWeight','bold');	%设置坐标刻度字体
% hold on;
 
%% 寻找路径
while It<Itmax  %针对需要优化路径
    It=It+1;
    totalPath = [];
    

     % 删除之前RRT树的绘制
        if ~isempty(treeHandles)
            delete(treeHandles);
            delete(lineHandles);
        end
        treeHandles = [];
      %RRT路径生成
for k1 = 1:size(pathPoint,1)-1
    startPoint = pathPoint(k1,:);
    goalPoint = pathPoint(k1+1,:);
 
%          [Path, newTreeHandles] = RRT(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
%          [Path, newTreeHandles] = AVSRRT(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
         [Path, newTreeHandles] = RRTstar(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
        if ~isempty(Path)  %判断是否为空数组，若不是，继续执行
            treeHandles = [treeHandles; newTreeHandles]; % 保存RRT树的句柄
        for k2 = 1:size(Path,1)-1
            h=line([Path(k2,1) Path(k2+1,1)],[Path(k2,2) Path(k2+1,2)],[Path(k2,3) Path(k2+1,3)],'LineWidth',1,'Color','r');
            treeHandles = [treeHandles; h]; % 保存路径线段的句柄
            lineHandles = [lineHandles; h]; % 保存每段线的句柄
   
        end
        totalPath = [totalPath;Path];   %拼接起来，刚开始为空
    end
end
hold on;

% 贝塞尔曲线拟合
N=length(totalPath);    %确定贝塞尔阶数（控制点个数-1）
M=N;  %贝塞尔曲线控制点的个数

%计算杨辉三角
t=zeros(N,N,N);

for i=1:N
    t(i,1) = 1;
    t(i,i) = 1;
end
if N>=3
    for i=3:N
        for j=2:i-1
            t(i,j) = t(i-1,j-1)+t(i-1,j);
        end
    end
end

% % 根据公式计算贝塞尔曲线
re=zeros(M,3);
for i=1:M
    step = i/M;
    for k=0:N-1
        re(i,1) = re(i,1) + (1-step)^(N-k-1)*totalPath(k+1,1)*step^k*t(N,k+1); %t替换为nchoosek(N-1,k)，不用计算杨辉三角了;
        re(i,2) = re(i,2) + (1-step)^(N-k-1)*totalPath(k+1,2)*step^k*t(N,k+1);
        re(i,3) = re(i,3) + (1-step)^(N-k-1)*totalPath(k+1,3)*step^k*t(N,k+1);%t替换为nchoosek(N-1,k)，不用计算杨辉三角了; 
    end 
end
 bezierHandle=plot3(re(:,1),re(:,2),re(:,3),'k','LineWidth',1);
%  z=legend('Original path','Optimized path');
%  set(z,'FontName','Times New Roman','FontSize',11,'FontWeight','normal')
% hold on;
% 判断贝塞尔曲线是否发生碰撞

    cubeFlagB = isCubeB(cubeInfo,N,re);   %长方体碰撞检测函数
    cylinderFlagB = isCylinderB(cylinderInfo,N,re);  %圆柱体碰撞检测函数
    sphereFlagB = isSphereB(sphereInfo,N,re);   %球形障碍物碰撞检测函数
    
    if cubeFlagB || cylinderFlagB ||sphereFlagB %或，有一个为1，说明发生碰撞，执行if中的continue，返回while下
        disp('检测到碰撞，进行下一次循环');
% %         打印调试信息
        disp(['CubeFlagB: ', num2str(cubeFlagB)]);
        disp(['CylinderFlagB: ', num2str(cylinderFlagB)]);
        disp(['SphereFlagB: ', num2str(sphereFlagB)]);
        disp(['这是第',num2str(It),'次循环。']);
        delete(lineHandles);
        delete(bezierHandle);
        continue;
    end
    disp('找到无碰撞路径，跳出循环');
   break
end
if It==Itmax
    Path = [];
    disp('路径规划失败');
    return;
end
%% 
% hold on;

%% 实际路径长度
T=totalPath;
sum=0;
for n=1:size(Path,1)-1
    sum=sum+norm(T(n+1,:)-T(n,:));
end
disp(['优化前路径长度=', num2str(sum)]);
%%  优化后路径长度
% sun=0;
% for n=1:size(Path,1)-1
%     sun=sun+norm(re(n+1,:)-re(n,:));
% end
% disp(['优化后路径长度=', num2str(sun)]);
toc