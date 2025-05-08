function randCoor = samplePoint(axisStart,axisLWH,goalPoint)
%  加入以目标点为导向作用的RRT
if rand<0.5
    randX = rand*axisLWH(1)+axisStart(1);
    randY = rand*axisLWH(2)+axisStart(2);
    randZ = rand*axisLWH(3)+axisStart(3);
    randCoor = [randX randY randZ];
    
else
    randCoor = goalPoint;  
end
 
end
% % 原始RRT
%     randX = rand*axisLWH(1)+axisStart(1);
%     randY = rand*axisLWH(2)+axisStart(2);
%     randZ = rand*axisLWH(3)+axisStart(3);
%     randCoor = [randX randY randZ]; 
% 
%  
% end
%  