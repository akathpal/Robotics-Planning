function [Status_ur, NewNode_ur] = ActionMoveUpRight(CurrentNode)
    NewNode_ur = CurrentNode;
    if NewNode_ur(2)<150 && NewNode_ur(1)<250
        Status_ur = 1;
        NewNode_ur(2)= NewNode_ur(2) + 1;
        NewNode_ur(1)= NewNode_ur(1) + 1;
    else
        Status_ur = 0;
    end
    