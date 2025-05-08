%% 清空变量
clear all;
close all;
clc;
tic
%% 定义变量

axisStart = [0 0 0];
axisLWH = [200 200 200];
it = 0;
itmax=100;
%定义障碍物
cubeInfo.exist = 0;
cylinderInfo.exist = 0;
sphereInfo.exist = 0;

pathPoint = [0 100 10;

             200 100 10];  %一系列的路径点
 
cubeInfo = createCubeObject(cubeInfo);       %创建长方体障碍物信息
cylinderInfo = createCylinderObject(cylinderInfo);  %创建圆柱障碍物信息
sphereInfo = createSphereObject(sphereInfo);    %创建球形障碍物信息
 
%% 画图
fig=figure ;
lineHandles = []; % 存储绘制的线句柄
treeHandles = []; % 存储RRT树的句柄
colorMatCube = [0.87 0.87 0.8];
colorMatCylinder = [0 1 0];
colorMatSphere = [0 0 1];
pellucidity = 1;    %透明度
hold on;
scatter3(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
scatter3(pathPoint(end,1),pathPoint(end,2),pathPoint(end,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
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
hold on;
 
%% 寻找路径
% while it<itmax
    it=it+1;
    totalPath = [];
    

      %RRT路径生成
for k1 = 1:size(pathPoint,1)-1
    startPoint = pathPoint(k1,:);
    goalPoint = pathPoint(k1+1,:);

  [Path, newTreeHandles] = RRT_connect(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
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
% hold on;

if it==itmax
%     Path = [];
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
toc