function [Status_ul, NewNode_ul] = ActionMoveUpLeft(CurrentNode)
    NewNode_ul = CurrentNode;
    if NewNode_ul(2)<150 && NewNode_ul(1)>0
        Status_ul = 1;
        NewNode_ul(2)= NewNode_ul(2) + 1;
        NewNode_ul(1)= NewNode_ul(1) - 1;
    else
        Status_ul = 0;
    end
    