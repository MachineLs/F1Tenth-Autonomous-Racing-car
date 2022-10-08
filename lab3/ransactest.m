clear all;
clc;

% read all 6 images
for imageIndex = 1:6
    fullimg = imread(strcat('image',int2str(imageIndex),'.jpg'));
    [row,column] = size(fullimg);
    
% Split the image and set each row and column for each one as [341,column
% size], different picture has different size

    blue = fullimg(1:floor(row/3),:);
%   size(blue)
    green = fullimg(floor(row/3) + 1:(2*floor(row/3)),:);
%   size(green)
    red = fullimg((2*floor(row/3))+2:row,:);
%   size(red)
    

  % Merging the R G B planes into one image
  
    rgbimage(:,:,1) = blue;
    rgbimage(:,:,2) = green;
    rgbimage(:,:,3) = red;
    imwrite(rgbimage, strcat('image',int2str(imageIndex),'-color.jpg'));
    
    
    im_align3(blue,green,red);

%     harris(blue)

    
    
    
    clear;% clear the image variable at the bottom to avoid the dimention bug
    
    
end