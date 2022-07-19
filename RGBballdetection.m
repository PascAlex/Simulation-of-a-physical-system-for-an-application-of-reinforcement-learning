function [xnew,ynew,x_pnew,y_pnew] = RGBballdetection(imageRGB,x0,y0,timestep,dimxPiatto,dimyPiatto)%,step)
% The function computes the ball-centroid cordinates and its velocity
% INPUT: - Image of the red ball on the black plate 
%        - old ball-centroid cordinates
%        - timestep of the simualtion
%        - plate size x and y 
% OUTPUT: - x and y coordinates of the ball centroid
%         - ball centroid velocity along x and y
    Rdetection = imageRGB(:,:,1);
    Rdetection1 = Rdetection > 200;
                
    [PixelPos] = regionprops('table',Rdetection1,'Centroid');
    PixelPos = PixelPos{1,1};
    xPixelnew = 128- PixelPos(1,1);
    yPixelnew = 128- PixelPos(1,2);



    xnew = (64-xPixelnew)*(dimxPiatto/128);
    ynew = (yPixelnew-64)*(dimyPiatto/128);
    x_pnew =  ((xnew -x0)/timestep);
    y_pnew =  ((ynew -y0)/timestep);

end