clear;
clc;

%read all 6 images

for imageIndex = 1:6
    fullimg = imread(strcat('image',int2str(imageIndex),'.jpg'));
    [row,column] = size(fullimg);
    
% Split the image and set each row and column for each one as [row/3,column
% size], different picture has different size

    blue = fullimg(1:floor(row/3),:);
%   size(blue)
    green = fullimg(floor(row/3) + 1:(2*floor(row/3)),:);
%   size(green)
    red = fullimg((2*floor(row/3))+2:row,:);
%   size(red)
  
  % Create an RGB image for each one
  
    rgbimage(:,:,1) = blue;
    rgbimage(:,:,2) = green;
    rgbimage(:,:,3) = red;
    imwrite(rgbimage, strcat('image',int2str(imageIndex),'-color.jpg'));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  % SSD shift and creat image
    tic
    [green_x,green_y,red_x,red_y,ssd] = im_align1(red,green,blue);
    disp(strcat('Image',int2str(imageIndex)));
    disp(strcat('SSD Green shift: [',int2str(green_x),',',int2str(green_y),']'));
    disp(strcat('SSD Red shift: [',int2str(red_x),',',int2str(red_y),']'));
    imwrite(ssd, strcat('image',int2str(imageIndex),'-ssd.jpg'));
    t = toc;
    time = ['The time is: ',num2str(t)];
    disp(time)
    
%   NCC shift and creat image
    tic
    [green_x,green_y,red_x,red_y,ncc] = im_align2(red,green,blue);
    disp(strcat('NCC Green shift: [',int2str(green_x),',',int2str(green_y),']'));
    disp(strcat('NCC Red shift: [',int2str(red_x),',',int2str(red_y),']'));
    imwrite(ncc, strcat('image',int2str(imageIndex),'-ncc.jpg'));
    t = toc;
    time = ['The time is: ',num2str(t)];
    disp(time)
   

    clear;% clear the image variable at the bottom to avoid the dimention bug
end