function [x, y] = harris(img)
    threshold = 1000000000;
    k = 0.04;
    sigma = 5;
    g = fspecial('Gaussian',[3 3],sigma);
    img1 = imfilter(img,g);
    img1 = double(img1);  
    Gx = [-1 0 1;-2 0 2;-1 0 1];
    Gy = Gx';
    Ix = imfilter(img1,Gx);
    Iy = imfilter(img1,Gy);
    Ix2 = Ix .* Ix;
    Iy2 = Iy .* Iy;
    Ixy = Ix .* Iy;

    Sx2 = imfilter(Ix2,g);
    Sy2 = imfilter(Iy2,g);
    Sxy = imfilter(Ixy,g);
    for x = 1 : size(Ix2,1)
        for y = 1 : size(Iy2,2)
            M = [Sx2(x,y) Sxy(x,y);Sxy(x,y) Sy2(x,y)];
            R(x,y) = det(M) - k * (trace(M) ^ 2);
        end
    end
    
    R = R > threshold;
    Corner_feature = zeros(1,2);
    for i = 1:size(R,1)
        for j = 1:size(R,2)
            if R(i,j) == 1
                Corner_feature = [Corner_feature;[i j]];
            end
        end
    end
    
    x = Corner_feature(:,1);
    y = Corner_feature(:,2);
    
    
    
%     R1 = feature_find(R,200);
%     R1;

end

          
