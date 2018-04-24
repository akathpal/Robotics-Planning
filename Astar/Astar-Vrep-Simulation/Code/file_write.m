clc;clear;close all;
fileID = fopen('output_vrep_final-data-2hz','w');
formatSpec = 'linear.x = %f, linear.y = %f, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = %f \n';
% load vel_x;
% load vel_y;
% load vel_z;
% load angular_x;
% load angular_y;
% load angular_z;
load final_output.mat
a = final_output;
%fprintf(fileID,formatSpec,vel_x,vel_y,angular_z);
k=1;
for i = 1:size(a,1)
  if i==10*k
  vel_x =a(i,2)*-1;
  vel_y =a(i,3)*-1;
  angular_z = a(i,4)*pi/180;
  k=k+1;
  fprintf(fileID,formatSpec,vel_x,vel_y,angular_z);
  end
  
end
fclose(fileID);