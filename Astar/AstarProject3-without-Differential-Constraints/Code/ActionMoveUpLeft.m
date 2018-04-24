function [Status_ul, NewNode_ul] = ActionMoveUpLeft(CurrentNode,res)
    NewNode_ul = CurrentNode;
    if NewNode_ul(2)<9.8 && NewNode_ul(1)>0.2
        Status_ul = 1;
        NewNode_ul(2)= NewNode_ul(2) + res;
        NewNode_ul(1)= NewNode_ul(1) - res;
    else
        Status_ul = 0;
    end
    