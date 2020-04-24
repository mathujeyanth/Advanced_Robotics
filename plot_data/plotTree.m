clc;
close all;
clear;
addpath('../../..')
%% Load data

Tree1 = importdata('1xyzTree.txt');
Tree2 = importdata('2xyzTree.txt');

plot3(Tree1(:,1),Tree1(:,2),Tree1(:,3),'*r',Tree2(:,1),Tree2(:,2),Tree2(:,3),'*g')
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
legend({'Tree1 nodes','Tree2 nodes'})
xlim([-5 5])
ylim([-5 5])
zlim([-5 5])

