function sphereFlagR = isSphereCollisionR(sphereInfo, pp, newCoor, step)

sphereFlagR = 0;
calcuDis = @(x,y) sqrt((x(1)-y(1))^2 + (x(2)-y(2))^2 + (x(3)-y(3))^2);

if sphereInfo.exist
    for k1 = 1:size(sphereInfo.centerX, 2)
        center = [sphereInfo.centerX(k1), sphereInfo.centerY(k1), sphereInfo.centerZ(k1)];
        
        % 计算路径的步数
        numSteps = ceil(norm(newCoor - pp) / step);  
        
        % 线性插值生成路径上的点
        for k2 = linspace(0, 1, numSteps)
            checkPoint = pp + k2 * (newCoor - pp);  % 线性插值
            
            % 碰撞检测
            if calcuDis(checkPoint, center) < sphereInfo.radius(k1)
                sphereFlagR = 1;
                return;
            end
        end
    end
end

end
