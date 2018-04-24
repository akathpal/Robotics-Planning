function [Status_ur, NewNode_ur] = ActionMoveUpRight(CurrentNode,res)
    NewNode_ur = CurrentNode;
    if NewNode_ur(2)<9.8 && NewNode_ur(1)<14.8
        Status_ur = 1;
        NewNode_ur(2)= NewNode_ur(2) + res;
        NewNode_ur(1)= NewNode_ur(1) + res;
    else
        Status_ur = 0;
    end
    