function []=myAnimationSonarPath(xActEst,yActEst,thetaActEst,X_array, Y_array, rp1,rp2,sonarInfo, obstaclesArray, OccGrid, gridorig, cellsize)
% to animate the motion of the UGV through assigned wayPoints
% Atilla Dogan, 20141101
% Modified by Manfred Huber to include Path Planning
% XX4378/XX5378 - Intro to UVS - Fall 2014
%
% rev 20150403: added the rectangle in front of UGV representing sonar
% sensor coverage

ugvWidth= 0.3556;
ugvLength=0.4635;
ugvShape = [-ugvLength/2 -ugvLength/2 ugvLength/2 ugvLength/2;
            ugvWidth/2 -ugvWidth/2 -ugvWidth/2 ugvWidth/2];

        
% sonar sensor coverage
sonarWidth  = sonarInfo.as;
sonarLength = sonarInfo.bs;
sonarOffset = sonarInfo.hs;

sonarShape = [sonarOffset    sonarOffset sonarOffset+sonarLength sonarOffset+sonarLength;
               sonarWidth/2 -sonarWidth/2 -sonarWidth/2 sonarWidth/2];

figure('units','normalized','position',[.1 .1 .75 .75])
handlePath = plot(yActEst.signals.values(:,1),xActEst.signals.values(:,1));
handleOccGrid = plot(gridorig(2), gridorig(1), gridorig(2)+size(OccGrid, 2)*cellsize(2), gridorig(1)+size(OccGrid, 1)*cellsize(1));
hold on;

plot(Y_array,X_array,'*r')

grid
xlabel('y [m]')
ylabel('x [m]')
% wayPOint placement
for ii = 1:length(X_array)
    my_circle(rp2,Y_array(ii),X_array(ii),1,1,'-r',1)
    my_circle(rp1,Y_array(ii),X_array(ii),1,1,'--b',1)
end
% obstacle Placements
temp =size(obstaclesArray);
noOfObst = temp(2);
for ii = 1:noOfObst
    my_circleFill(obstaclesArray(3,ii),obstaclesArray(2,ii),obstaclesArray(1,ii),1,1,'r')
end
vAxis = axis;
delete(handlePath)
axis image

% Occupancy grid drawing
for ix = 1:size(OccGrid, 1),
    for iy = 1:size(OccGrid, 2),
        if (OccGrid(ix, iy) == 1)
            rectangle('Position', [gridorig(2)+(iy-1)*cellsize(2), gridorig(1)+(ix-1)*cellsize(1), cellsize(2), cellsize(1)], 'FaceColor', 'r');
        end;
        if (OccGrid(ix, iy) == 2)
            rectangle('Position', [gridorig(2)+(iy-1)*cellsize(2), gridorig(1)+(ix-1)*cellsize(1), cellsize(2), cellsize(1)], 'FaceColor', 'y');
        end;
        if (OccGrid(ix, iy) == 3)
            rectangle('Position', [locy, locx, cellsize(2), cellsize(1)],[gridorig(2)+(iy-1)*cellsize(2), gridorig(1)+(ix-1)*cellsize(1), cellsize(2), cellsize(1)], 'FaceColor', 'g');
        end;
    end;
end;
rectangle('Position', [gridorig(2), gridorig(1), size(OccGrid, 2)*cellsize(2), size(OccGrid, 1)*cellsize(1)]);

axis([vAxis(1)-2*max(rp1,rp2) vAxis(2)+2*max(rp1,rp2) vAxis(3)-2*max(rp1,rp2) vAxis(4)+2*max(rp1,rp2)])
ugvHandle = fill(ugvShape(2,:),ugvShape(1,:),'b');
sonarHandle = fill(sonarShape(2,:),sonarShape(1,:),'m');
set(sonarHandle,'facealpha',0.4); 

tailHandle = plot(yActEst.signals.values(1,1),xActEst.signals.values(1,1));
tailL = 500;

for iii=1:length(xActEst.signals.values(:,1))
 delete(ugvHandle)
 delete(sonarHandle)
 delete(tailHandle)
 ugvShapeRot = rotMat2D(thetaActEst.signals.values(iii,1))*ugvShape;
 ugvShapeMov = [ugvShapeRot(1,:)+xActEst.signals.values(iii,1);ugvShapeRot(2,:)+yActEst.signals.values(iii,1)];

 sonarShapeRot = rotMat2D(thetaActEst.signals.values(iii,1))*sonarShape;
 sonarShapeMov = [sonarShapeRot(1,:)+xActEst.signals.values(iii,1);sonarShapeRot(2,:)+yActEst.signals.values(iii,1)];

 
 ugvHandle = fill(ugvShapeMov(2,:),ugvShapeMov(1,:),'b');
 sonarHandle = fill(sonarShapeMov(2,:),sonarShapeMov(1,:),'m');
 set(sonarHandle,'facealpha',0.4);
 tailHandle = plot(yActEst.signals.values([iii:-1:max(iii-tailL,1)],1),xActEst.signals.values([iii:-1:max(iii-tailL,1)],1),':b','Linewidth',2);

 pause(0.01)
 
end

function R = rotMat2D(angle)
R = [cos(angle) -sin(angle);
     sin(angle)  cos(angle)];

