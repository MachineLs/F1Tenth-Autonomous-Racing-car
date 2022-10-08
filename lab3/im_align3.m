function RGB = im_align3(b,g,r)
% clear,clc;

% R_part = imread('image1_RGB\image1-red.jpg');
% G_part = imread('image1_RGB\image1-green.jpg');
% B_part = imread('image1_RGB\image1-blue.jpg');

[H,W] = size(b); % Height, Weight

[Xb,Yb] = harris(b*1); %[count , X, Y]
[Xr,Yr] = harris(r*0.4);
[Xg,Yg] = harris(g*0.4);

iters = 1000;
threshold = 2;

[sXr,sYr]= ransac_t(Xb,Yb,Xr,Yr, threshold, iters);
 
[HRRed_img_red] = shift(H,W,sXr,sYr,r);


[sXg,sYg]= ransac_t(Xb,Yb,Xg,Yg, threshold, iters);
[HRRed_img_green] = shift(H,W,sXg,sYg,g);


% eval(['NCCed_img',num2str(n),'=','zeros(H,W)',';'])



% RGB(:,:,3) = b;
% RGB(:,:,2) = HRRed_img_green;
% RGB(:,:,1) = HRRed_img_red;
% figure;
% imshow(RGB);
RGB = cat(3,HRRed_img_red,HRRed_img_green,b);

function [shifted] = shift(H,W,X,Y,img)
    shifted = zeros(H,W);
    trans = [1 0 Y; 0 1 X; 0 0 1];

    for iH = 1:H
        for j = 1:W
            temp = [iH;j;1];
            temp = trans*temp;
            x = temp(1,1);
            y = temp(2,1);
            if (x <= H) && (y <= W) && (x>=1) && (y>=1)
                    shifted(x,y) = img(iH,j);
            end
        end
    end
    
end

function [best_total,best_tranX, best_tranY] = ransac_t(X0,Y0,X1,Y1, threshold, iters)

    best_total = 0;


    while iters > 0
        index1 =  ceil(rand*200);
        index2 =  ceil(rand*200);
        tranx = X0(index1)-X1(index2);
        trany = Y0(index1)-Y1(index2); 
        
        for m = 1:200
            Xpb = X1(m)+tranx;
            Ypb = Y1(m)+trany;
            total = 0;
            
            for n = 1:200
                if abs(Xpb-X0(n)) < threshold && abs(Ypb-Y0(n)) < threshold
                   
                    total = total+1;
                    
                end
            end
            if total > best_total
                        best_total = total;
                        best_tranX = tranx;
                        best_tranY = trany;
            end
            iters = iters-1;
        end
        
        
    end    

end



end