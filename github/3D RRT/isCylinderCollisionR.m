function cylinderFlagR = isCylinderCollisionR(cylinderInfo, pp,newCoor, step)
%% 圆柱体碰撞检测函数，当发生碰撞的时候返回1
 
cylinderFlagR = 0;
calcuDis = @(x, y) sqrt((x(1) - y(1))^2 + (x(2) - y(2))^2);  % 计算平面内距离

if cylinderInfo.exist
    for k1 = 1:size(cylinderInfo.X, 2)
        zMin = cylinderInfo.Z(k1);
        zMax = zMin + cylinderInfo.height(k1);
        
        % 计算路径的步数
        numSteps = ceil(norm(newCoor - pp) / step);  
        
        % 线性插值生成路径上的点
        for k2 = linspace(0, 1, numSteps)
            checkPoint = pp + k2 * (newCoor - pp);  % 线性插值
            
            % 检查 XY 平面距离是否小于圆柱体半径，且 z 轴高度在范围内
            if calcuDis(checkPoint(1:2), [cylinderInfo.X(k1), cylinderInfo.Y(k1)]) < cylinderInfo.radius(k1) && ...
               zMin < checkPoint(3) && checkPoint(3) < zMax
                cylinderFlagR = 1;
                return;
            end
        end
    end
end

end
