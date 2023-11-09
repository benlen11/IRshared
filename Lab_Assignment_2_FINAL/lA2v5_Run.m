%clear all
clc
clf
clear
close all

hold on

% setup workspace
axis([-3,3,-3,3,-0.1,3]);
set(gca, 'Projection', 'orthographic');
axis equal;
  
%Code
lA2v5_Clean.Environment
lA2v5_Clean.Animate

disp('Collision check simulation')
pause
%%
lA2v5_CleanCol.Environment
lA2v5_CleanCol.Animate
