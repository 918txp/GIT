function drawCylinderObject(cylinderInfo,colorMatCylinder,pellucidity)
 
if cylinderInfo.exist
    %  第一个参数是圆柱体的底部圆心坐标值，第二个参数是圆柱体直径，第三个参数是圆柱高度
    %  第四个参数是透明度，第五个参数是颜色矩阵
    for k1 = 1:size(cylinderInfo.X,2)
        coor = [cylinderInfo.X(k1) cylinderInfo.Y(k1) cylinderInfo.Z(k1)];
        diameter = cylinderInfo.radius(k1)*2;
        height = cylinderInfo.height(k1);
        facealpha = pellucidity;
        color = colorMatCylinder;
        plotcylinder(coor,diameter,height,facealpha,color)
        
    end
    
end
 
end