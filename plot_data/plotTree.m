clc;
close all;
clear;
addpath('../../..')
%% Load data

Tree = importdata('xyzTree.txt');
Path = importdata('xyzTreePath.txt');

%figure(1)
%plot(A)

figure(2)
hold on
plot(Tree(:,1),Tree(:,3),'*')
plot(Path(:,1),Path(:,3),'*g')
plot(-0.36,0.262,'*r')
xlabel('x axis')
ylabel('z axis')
legend({'Tree nodes','Path nodes','Goal node'})
hold off