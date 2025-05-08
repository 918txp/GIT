function parentIndex = RewriteFunction(T,newCoor,r,parentIndex)
%% 重写函数，将新点newCoor重新连接到代价最小
 
% 下面是寻找到潜在的父节点，在变量potentialParent里
potentialParent = -ones(1,size(T.x,2));
count = 1;
for k1 = 1:size(T.x,2)
    if CalcuDistance(newCoor,[T.x(k1),T.y(k1),T.z(k1)])<r
        potentialParent(count) = k1;
        count = count+1;
    end
end
potentialParent(potentialParent==-1)=[];
 
%迭代寻找最小代价，并重新选择新节点newCoor的父节点，并把parentIndex改为新父节点对应的索引
for k2 = 1:size(potentialParent,2)
    % potentialParent(k2)潜在父节点的序号
    pp = [T.x(potentialParent(k2)),T.y(potentialParent(k2)),T.z(potentialParent(k2))];
    p = [T.x(parentIndex),T.y(parentIndex),T.z(parentIndex)];
    if (CalcuDistance(pp,newCoor)+T.cost(potentialParent(k2)))<(CalcuDistance(p,newCoor)+T.cost(parentIndex))
        parentIndex = potentialParent(k2);
    end
end