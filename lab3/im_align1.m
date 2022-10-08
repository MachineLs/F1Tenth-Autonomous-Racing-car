function [green_x,green_y,red_x,red_y,ssd] = im_align1(r,g,b)


  % Get x and y size from b
  [px,py] = size(b);
  
  % In order to reduce the noise and make it look nice wo can slice the img, set the row as px-28, column as py-28
  % and retain most useful part
  edge_cut = 28;
  
  b_withoutedge = b(edge_cut:px-edge_cut,edge_cut:py-edge_cut);
  
%   size(newb)
 
  % Set the initial SSD to the maximum possible value
  
  ssdg = inf;
  ssdr = inf;
  
  % search window size as 15*15
  
  for i = -15:15
    for j = -15:15
      g_ssd = g(edge_cut+i:px-edge_cut+i , edge_cut+j:py-edge_cut+j);
      r_ssd = r(edge_cut+i:px-edge_cut+i , edge_cut+j:py-edge_cut+j);
      
      offg = (b_withoutedge - g_ssd).^2;
      offr = (b_withoutedge - r_ssd).^2;
      
      temp_ssdg = sum(sum(offg));
      temp_ssdr = sum(sum(offr));
      
      if temp_ssdg <= ssdg
        ssdg = temp_ssdg;
        green_x = i;
        green_y = j;
      end
      
      if temp_ssdr <= ssdr
        ssdr = temp_ssdr;
        red_x = i;
        red_y = j;
      end
    end
  end
  
  % cut the edge for g and r and implement ssd shift
  g_ssd = g(edge_cut+green_x:px-edge_cut+green_x , edge_cut+green_y:py-edge_cut+green_y);
  r_ssd = r(edge_cut+red_x:px-edge_cut+red_x , edge_cut+red_y:py-edge_cut+red_y);
  % merger the image
  ssd = cat(3,r_ssd,g_ssd,b_withoutedge);
  
  
end