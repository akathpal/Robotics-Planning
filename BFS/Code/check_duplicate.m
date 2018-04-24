function visited = check_duplicate(id,NewNode_id)
%   n = size(id,2);
%   for i=1:1:n
%       if(NewNode_id == id(i))
%           visited = 1;
%           break;
%       else
%           visited = 0;
%       end
%   end
    if(any(id==NewNode_id))
        visited =1;
    else
        visited = 0;
    end