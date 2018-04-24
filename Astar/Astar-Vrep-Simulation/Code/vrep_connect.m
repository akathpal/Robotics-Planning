clc;
clear;
close all;
res =0.25;
vrep=remApi('remoteApi');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');

    
    %% Generating Object Handles for getting object properties from vrep
    [s,left_wheel] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    [ s,right_wheel]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    %% Dummy is reference frame
    [ s,reference_frame]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    [ s,turtlebot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end

    %% Code
    %% getting Position
    [s,start_position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    start = double([start_position(1,1),start_position(1,2)]);
    
    %% getting path from Astar without differential constraints
    final_path = turtlebot_astar(start,res);
    s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);
    if s ~= 0
       disp("Error");
    end

    for i=2:size(final_path,1)

        [~,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        if s ~= 0
            disp("Error");
        end     
        position = double(position);
        theta_req = atan2(final_path(i,2)-position(1,2),final_path(i,1)-position(1,1));
        disp(i);

        
        %% Orienrtation_correction
        while  abs(theta(3) - theta_req)> 0.05
            if theta(3) < theta_req 
                s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-2,vrep.simx_opmode_oneshot_wait);
                if s ~= 0
                    disp("Error");
                end
                s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait);
                if s ~= 0
                    disp("Error");
                end
                
            else
                s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,2,vrep.simx_opmode_oneshot_wait);
                if s ~= 0
                    disp("Error");
                end
                s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait);
                if s ~= 0
                    disp("Error");
                end
            end
                
            [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
            if s ~= 0
                disp("Error");
            end          
        end
        
        [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        if s ~= 0
           disp("Error");
        end
        position = double(position);
        
        
        k=1;
        %% Position Correction
        while 1
            k=k+1;
            [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
            if s ~= 0
               disp("Error");
            end
            position = double(position);
            
           
            if abs(position(1,1) - final_path(i,1)) < 0.1 &&  abs(position(1,2) - final_path(i,2)) < 0.1
                break;
            end
    
            s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,10,vrep.simx_opmode_oneshot_wait);
            if s ~= 0
               disp("Error");
            end
            
            s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,10,vrep.simx_opmode_oneshot_wait);
            if s ~= 0
               disp("Error");
            end
            
            %% check orientation if k>10
            if k>10
                disp("Here")
                break;
            end
        end
        
    end
 
    %% Location reached, stop robot
    disp("Location Reached");
    s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);
    if s ~= 0
        disp("Error");
    end
    s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);
    if s ~= 0
       disp("Error");
    end
    
    vrep.simxFinish(-1);
end

vrep.delete(); % call the destructor!

disp('Program ended');

