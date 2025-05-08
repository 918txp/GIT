function sphereFlagB = isSphereB(sphereInfo,N,re)
 
sphereFlagB = 0;
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
 
 
if sphereInfo.exist
   for k1 = 1:size(sphereInfo.centerX,2)
       center = [sphereInfo.centerX(k1) sphereInfo.centerY(k1) sphereInfo.centerZ(k1)];
       
       for k2 = 1:N
            checkpoint = [re(k2,1),re(k2,2),re(k2,3)];
           
            if calcuDis(checkpoint,center)<sphereInfo.radius(k1)
                sphereFlagB = 1;
                return;
            end
            
        end
       
   end
end
 
end
 