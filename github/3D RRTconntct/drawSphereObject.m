function drawSphereObject(sphereInfo,colorMatSphere,pellucidity)
 
if sphereInfo.exist
    for k1 = 1:size(sphereInfo.centerX,2)
        xCoor = sphereInfo.centerX(k1);
        yCoor = sphereInfo.centerY(k1);
        zCoor = sphereInfo.centerZ(k1);
        radius = sphereInfo.radius(k1);
        
        [x,y,z] = sphere(50);
        mesh(x*radius+xCoor,y*radius+yCoor,z*radius+zCoor,'FaceColor',colorMatSphere,'EdgeColor','none','FaceAlpha',pellucidity);
        
        
    end 
    
end
 
end