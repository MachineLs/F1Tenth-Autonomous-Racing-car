function [C] = feature_find(A,n)
%% This function help to find the TOP 200 features and its coordinate
    
%  A = [1 2 3;4 5 6;7 8 9]
%  n = 3 % Top 3 features from A, return its coordinate

    C = []; % C is x and y coordinate of these coordinates
    for k = 1:n
    max_value = max(max(A))
    [i,j]=find(A==max_value);
    C = [C;[i,j]];
    A(i,j) = -inf;
    end
end