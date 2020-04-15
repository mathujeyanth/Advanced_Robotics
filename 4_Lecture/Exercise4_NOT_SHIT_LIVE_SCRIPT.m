clc;
clear;
close all;
%%

q=[pi/15 pi/4 pi/4];

C_t=[0 1 0]'; % Constraint matrix: [x,y,theta], constrained in y direction
%% q initial
q=(rand(1,3)-0.5)*2*pi;
pos = directKin(q);
TCPpos = pos(4,:);
dy_err = abs(TCPpos(2)-1);
dx_err = abs(TCPpos(1)-1);
counter1=0;
tic %Start timer
while dy_err>0.05 || dx_err>0.05 
   dq=(rand(1,3)-0.05)*2*pi;
   q_new=q+dq;
   q_new=sign(q_new).*mod(q_new,pi);
   new_pos = directKin(q_new);
   new_dx_err = abs(new_pos(4,1)-1);
   new_dy_err = abs(new_pos(4,2)-1);
   if new_dy_err < dy_err && new_dx_err < dx_err 
       q=q_new;
       dy_err = new_dy_err;
       dx_err = new_dx_err;
   end
   counter1=counter1+1;
end
%Initial config found
q_init=q;
disp('Initial config found: ')
toc %Stop timer and display time
pos_init=directKin(q)
Iteration_to_find_init=counter1
draw_manipulator(q) 

i = 2; %Matlab is 1-indexed and first index is used for intial
dt = 1;
Tree(1,:)=[1,q_init,pos_init(4,:)]; %Save index, q_init and TCP-pos.
counter2 = 0;
maxJointStep = deg2rad(10);%Define max joint step, 10 degrees converted to rad
tic %Start new timer
while i < 500
    %index=randi(size(Tree,1));
    index=i-1; %Directed RRT - thus not use random index, but previous
    q=Tree(index,2:4);
    dq=(rand(1,3)-0.01)*2*pi;
    q_rand = q + dq;
    q_rand=sign(q_rand).*mod(q_rand,pi);
    q_s=q_rand; %simple conversion - only needed q_rand
    %Dont know what the followin 3 lines does
    %q_dir = (q_rand-q)/vecnorm(q_rand-q);
    %q_s = q + q_dir * dt;
    %q_s=sign(q_s).*mod(q_s,pi);
    
    %Find TCP-pos of q_s
    pos_s = directKin(q_s);
    if( (0.95<pos_s(4,2)) && (pos_s(4,2) < 1.05) && (abs(q_s(1)-q(1))<maxJointStep)  && (abs(q_s(2)-q(2))<maxJointStep) && (abs(q_s(3)-q(3))<maxJointStep))%Check if its within y=1 +- 0.05 and that q doesn't change more than maxJointStep
        %Limit movement in x-direction (If-statement below). 
        %Range: OldPos-0.05 < NewPos < OldPos, thus less than OldPos but not too much
        if( (pos_s(4,1) > Tree(size(Tree,1),5)-0.05) && (pos_s(4,1) < Tree(size(Tree,1),5)) )
            Tree(i,:)=[i,q_s,pos_s(4,:)];
            i=i+1;
            if (pos_s(4,1) < -1) %End position reached
                break
            end
        end
    end
    counter2 = counter2+1;
end


disp('Iterations to move from start to goal and steps in Tree')
toc %Stop timer
Iteration_to_find_path=counter2
Tree_size=i

for i=1:size(Tree,1)
    fig(2)=figure(2);
    hold on
    cla
    pbaspect([3 1.5 1])
    plot([-1.5 1.5],[1 1],['--k'])
    p=directKin(Tree(i,2:4));
    for i = 1:length(p)
        draw_circle(0.01,p(i,:),'k');
    end
    for i=1:length(p)-1
        plot([p(i,1) p(i+1,1)],[p(i,2) p(i+1,2)],'-ok')
    end
    axis([-1.5 1.5 0 1.5]);
    box on
    pause(0.05);
    hold off
end


%% Functions
% function pos=directKin(q)
%     a1=1;a2=0.7;a3=0.2;
%     p=[0 0;0 0;0 0;0 0];
%     p(2,:)=a1*[cos(q(1)) sin(q(1))];
%     p(3,:)=p(2,:)+a2*[cos(q(1)+q(2)) sin(q(1)+q(2))];
%     p(4,:)=p(3,:)+a3*[cos(q(1)+q(2)+q(3)) sin(q(1)+q(2)+q(3))];
%     pos=p;
% end
% 
function draw_manipulator(q)
    fig(1)=figure(1);
    hold on
    pbaspect([3 1.5 1])
    plot([-1.5 1.5],[1 1],['--k'])
    p=directKin(q);
    for i = 1:length(p)
        draw_circle(0.01,p(i,:),'k');
    end
    for i=1:length(p)-1
        plot([p(i,1) p(i+1,1)],[p(i,2) p(i+1,2)],'-ok')
    end
    axis([-1.5 1.5 0 1.5]);
    box on
    hold off
end
% 
% 
% function h = draw_circle(r,c,colour)
%     th = linspace(0,2*pi);
%     x = r*sin(th)+c(1);
%     y = r*cos(th)+c(2);
%     h=fill(x,y,colour);
% end
