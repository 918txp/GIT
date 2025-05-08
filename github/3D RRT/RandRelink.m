function T = RandRelink(T,newCoor,cubeInfo,cylinderInfo,sphereInfo,step,r)
%% 随机重连，将新节点newCoor周围节点的父节点尝试改为新节点，若代价小于原来的代价值，则确认更改
 
% 寻找需要需要修改父节点的节点放入potentialParent里面。
potentialParent = -ones(1,size(T.x,2));
count = 1;
for k1 = 1:size(T.x,2)
    if CalcuDistance(newCoor,[T.x(k1),T.y(k1),T.z(k1)])<r
        potentialParent(count) = k1;
        count = count+1;
    end
end
potentialParent(potentialParent==-1)=[];
 
% 找到新节点的父节点集合，存在变量Index里
Index = -ones(1,size(T.x,2));
index = T.pre(end);
count = 1;
while T.pre(index)~=0
    Index(count) = index;
    index = T.pre(index);
    count = count+1;  
end
Index(Index==-1) = [];
potentialParent(ismember(potentialParent,Index)==1) = [];  %把需要修改父节点的节点集合，剔除新节点的父节点，以免出现环
 
% 根据代价决定是否修改父节点
for k2 = 1:size(potentialParent,2)
    pp = [T.x(potentialParent(k2)),T.y(potentialParent(k2)),T.z(potentialParent(k2))];
    if T.cost(potentialParent(k2))>( T.cost(end)+ CalcuDistance(pp,newCoor))
        if CollisionDetection(cubeInfo,cylinderInfo,sphereInfo,pp,newCoor,step)
%           cubeFlagR = isCubeCollisionR(cubeInfo, pp, newCoor, step);   %长方体碰撞检测函数
%           cylinderFlagR = isCylinderCollisionR(cylinderInfo, pp, newCoor, step);  %圆柱体碰撞检测函数
%           sphereFlagR = isSphereCollisionR(sphereInfo,pp, newCoor, step);   %球形障碍物碰撞检测函数
%                if cubeFlagR || cylinderFlagR || sphereFlagR 
%                continue;
%                end
            T.pre(potentialParent(k2)) = size(T.x,2)+1;
            T.cost(potentialParent(k2)) = CalcuDistance(newCoor,pp);
       end
   end
end