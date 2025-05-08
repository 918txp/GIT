function cubeFlagB = isCubeB(cubeInfo,N,re)
%% 优化后长方体碰撞检测函数，如果发生碰撞则返回1
 
cubeFlagB = 0;
 
if cubeInfo.exist
    for k1 =1: 7
        xMin = cubeInfo.axisX(k1);
        xMax = cubeInfo.axisX(k1)+cubeInfo.length(k1);
        yMin = cubeInfo.axisY(k1);
        yMax = cubeInfo.axisY(k1)+cubeInfo.width(k1);
        zMin = cubeInfo.axisZ(k1);
        zMax = cubeInfo.axisZ(k1)+cubeInfo.height(k1);
            
        for k2 = 1:N
             checkpoint = [re(k2,1),re(k2,2),re(k2,3)];
             
             if (xMin<checkpoint(1) && checkpoint(1) < xMax) && (yMin<checkpoint(2) && checkpoint(2) < yMax) && (zMin<checkpoint(3) && checkpoint(3) < zMax)
                cubeFlagB = 1;
                return;
            end
        end  
    end
        
 end
    
    
end
            
 