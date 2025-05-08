function [tree, newCoor, preIndex, success] = extendTree(tree, randCoor, step, Thr, cubeInfo, cylinderInfo, sphereInfo)
    calcuDis = @(x,y) sqrt((x(1)-y(1))^2 + (x(2)-y(2))^2 + (x(3)-y(3))^2); % 距离计算函数
     newCoor = []; % 初始化 nearCoor
    success = false;

    %% 寻找树上最近点
    [nearCoor, preIndex] = findNearPoint(randCoor, tree);
    if isempty(nearCoor) || isempty(randCoor)
        % 如果 nearCoor 或 randCoor 为空，返回失败
        return;
    end

    %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor, randCoor, step);

    %% 碰撞检测
    if ~isCubeCollision(cubeInfo, nearCoor, newCoor, step) && ...
       ~isCylinderCollision(cylinderInfo, nearCoor, newCoor, step) && ...
       ~isSphereCollision(sphereInfo, nearCoor, newCoor, step)
        %% 将新点插入树中
        tree.x(end+1) = newCoor(1);
        tree.y(end+1) = newCoor(2);
        tree.z(end+1) = newCoor(3);
        tree.pre(end+1) = preIndex;
        success = true;
    end
end