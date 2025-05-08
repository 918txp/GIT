function [nearCoor,parentIndex] = FindNearstPoint(T,randCoor)
%% 遍历整个树，寻找距离随机点randCoor最近的点，标记为nearestCoor
tempDis = inf;
parentIndex = -1;
for k1 = 1:size(T.x,2)
    dis = CalcuDistance(randCoor,[T.x(k1),T.y(k1),T.z(k1)]);
    if dis<tempDis
        tempDis = dis;
        parentIndex = k1;
    end
end
nearCoor = [T.x(parentIndex),T.y(parentIndex),T.z(parentIndex)];