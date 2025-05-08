function cubeFlagR = isCubeCollisionR(cubeInfo, pp,newCoor, step)
%% 长方体碰撞检测函数，如果发生碰撞则返回1
 
cubeFlagR = 0;

if cubeInfo.exist
    for k1 = 1:size(cubeInfo.axisX, 2)  % 确保适应动态长方体数量
        xMin = cubeInfo.axisX(k1);
        xMax = xMin + cubeInfo.length(k1);
        yMin = cubeInfo.axisY(k1);
        yMax = yMin + cubeInfo.width(k1);
        zMin = cubeInfo.axisZ(k1);
        zMax = zMin + cubeInfo.height(k1);
        
        % 计算路径的步数
        numSteps = ceil(norm(newCoor - pp) / step);  
        
        % 线性插值生成路径上的点
        for k2 = linspace(0, 1, numSteps)
            checkPoint = pp + k2 * (newCoor - pp);  % 线性插值
            
            % 检测点是否在长方体的范围内
            if (xMin < checkPoint(1) && checkPoint(1) < xMax) && ...
               (yMin < checkPoint(2) && checkPoint(2) < yMax) && ...
               (zMin < checkPoint(3) && checkPoint(3) < zMax)
                cubeFlagR = 1;
                return;
            end
        end
    end
end

end
