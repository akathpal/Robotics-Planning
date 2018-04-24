function [a,loc] = member(t,list)
%     %m = arrayfun( @(x) isequal( t, x.node ), list );
% %     l = {list(1:end).node}';
% %     m = cell2mat(l);
%     
%     m = [list(1:end).node];
%     temp = reshape(m,[2,size(m,2)/2]);
%     m = temp';
%     
%     if isempty(m)
%         a=0;loc=0;
%     else
%     [a,loc] = ismember(t,m,'rows');
%     end
% %     if a
% %         loc = find(m==1);
% %     else 
% %         loc = 0;
% %     end
    [a,loc] = ismember(t,list,'rows');
end