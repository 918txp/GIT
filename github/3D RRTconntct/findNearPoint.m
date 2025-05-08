% function [nearCoor, preIndex] = findNearPoint(randCoor, tree)
%   calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
% minDist = Inf;
%     for i = 1:length(tree.x)
%         dist = calcuDis([tree.x(i), tree.y(i), tree.z(i)], randCoor);
%         if dist < minDist
%             minDist = dist;
%             nearCoor = [tree.x(i), tree.y(i), tree.z(i)];
%             preIndex = i;
%         end
%     end
% end
% function  [nearCoor,preIndex] = findNearPoint(randCoor,T)
%  
% tempDis = inf;
% 
% calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
%  
% for k1 = 1:size(T.x,2)
%     dis = calcuDis([T.x(k1) T.y(k1) T.z(k1)],randCoor);
%     if tempDis>dis
%         tempDis = dis;
%         index = k1;
%     end    
%  
% end
%  
% nearCoor = [T.x(index) T.y(index) T.z(index)];
% preIndex = index;
%  
% end
function [nearCoor, preIndex] = findNearPoint(randCoor, tree)
    calcuDis = @(x,y) sqrt((x(1)-y(1))^2 + (x(2)-y(2))^2 + (x(3)-y(3))^2); % 距离计算函数
    minDist = Inf;
    nearCoor = []; % 初始化 nearCoor
    preIndex = 0;  % 初始化 preIndex
    if isempty(tree.x) || isempty(tree.y) || isempty(tree.z)
        % 如果树为空，返回默认值
        return;
    end
    for i = 1:length(tree.x)
        dist = calcuDis([tree.x(i), tree.y(i), tree.z(i)], randCoor);
        if dist < minDist
            minDist = dist;
            nearCoor = [tree.x(i), tree.y(i), tree.z(i)];
            preIndex = i;
        end
    end
end