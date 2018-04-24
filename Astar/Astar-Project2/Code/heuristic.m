function h = heuristic(Curr_node,Goal_node)
    x1 = Curr_node(1);
    y1 = Curr_node(2);
    x2 = Goal_node(1);
    y2 = Goal_node(2);
    a = max(abs(x1-x2),abs(y1-y2));
    b = min(abs(y1-y2),abs(x1-x2));
    h = b*14 + (a-b)*10;
    
end
