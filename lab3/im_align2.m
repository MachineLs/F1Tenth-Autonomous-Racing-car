function [green_x,green_y,red_x,red_y,ncc] = im_align2(r,g,b)
    
  % Get x and y size from b
    [px,py] = size(b);
  
    offset = 28;
  
  % In order to reduce the noise and make it look nice wo can slice the img, set the row as px-28, column as py-28
  % and retain most useful part
  
    b_withoutedge = b(offset:px-offset,offset:py-offset);
    b_withoutedge = double(b_withoutedge);
    
    % Set the initial SSD to the maximum possible value
    
    ncc_g = -inf;
    ncc_r = -inf;
    
    % search window size as 15*15
    for i = -15:15
        for j = -15:15
            new_g = g(offset+i:px-offset+i , offset+j:py-offset+j);
            new_r = r(offset+i:px-offset+i , offset+j:py-offset+j); 
            new_g = double(new_g);
            new_r = double(new_r);
        
            first = new_g./norm(new_g);
            second = b_withoutedge./norm(b_withoutedge);
            temp_cc_g = dot(first,second);
            temp_cc_g = sum(temp_cc_g);
           
            first_2 = new_r./norm(new_r);
            second_2 = b_withoutedge./norm(b_withoutedge);
            temp_cc_r = dot(first_2,second_2);
            temp_cc_r = sum(temp_cc_r);
            
            if temp_cc_g >= ncc_g
                ncc_g = temp_cc_g;
                green_x = i;
                green_y = j;
            end
            
            if temp_cc_r >= ncc_r
                ncc_r = temp_cc_r;
                red_x = i;
                red_y = j;
            end
        end
    end
    
  % cut the edge for g and r and implement ncc shift
    g_ncc = g(offset+green_x:px-offset+green_x , offset+green_y:py-offset+green_y);
    r_ncc = r(offset+red_x:px-offset+red_x , offset+red_y:py-offset+red_y);
  % merger the image
    ncc = cat(3,r_ncc,g_ncc,b_withoutedge);
end