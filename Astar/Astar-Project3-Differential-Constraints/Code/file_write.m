%% Generating the output file
clc;clear;close all;
fileID = fopen('final_output','w');
formatSpec = 'linear.x = %f, linear.y = %f, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = %f \n';
load v_x.mat;
load v_y.mat;
load theta.mat;

for i = 1:numel(v_x)
    fprintf(fileID,formatSpec,v_x(1,i),v_y(1,i),theta(1,i));
end
fclose(fileID);