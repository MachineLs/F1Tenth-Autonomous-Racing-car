function [C] = findTopNfeature(A, n)

%     A = [1 2 3;9 5 6;7 8 9]
%     n = 6;
    count = 0;
    C = [];
    for i = 1:size(A,1)
        for j = 1:size(A,2)
            max_value = max(max(A))
            [i j] = find(A==max_value);
            
            C = [C;[i j]];
            
            A(i,j) = 0
            count = count + 1;

            if count >= n 
                break;
            end
        end

        if count >= n
            break;
        end
    end
    
end


% [AS,pos] = sort(A(:),'descend');
% pos
% AS

