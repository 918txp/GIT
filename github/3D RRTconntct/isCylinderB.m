function cylinderFlagB = isCylinderB(cylinderInfo,N,re)
%% 优化后圆柱体碰撞检测函数，当发生碰撞的时候返回1
 
cylinderFlagB = 0;
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
 
if cylinderInfo.exist
    for k1 = 1:size(cylinderInfo.X,2)
        zMin = cylinderInfo.Z(k1);
        zMax = zMin+cylinderInfo.height(k1);
        
        for k2 = 1:N
             checkpoint = [re(k2,1),re(k2,2),re(k2,3)];

            if calcuDis(checkpoint(1:2),[cylinderInfo.X(k1) cylinderInfo.Y(k1)])<cylinderInfo.radius(k1) && zMin<checkpoint(3) && checkpoint(3) < zMax
                cylinderFlagB = 1;
                return;
            end
            
        end
        
    end
    
end
 
 
end
 