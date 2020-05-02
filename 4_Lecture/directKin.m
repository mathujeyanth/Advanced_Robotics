function pos=directKin(q)
    a1=1;a2=0.7;a3=0.2;
    p=[0 0;0 0;0 0;0 0];
    p(2,:)=a1*[cos(q(1)) sin(q(1))];
    p(3,:)=p(2,:)+a2*[cos(q(1)+q(2)) sin(q(1)+q(2))];
    p(4,:)=p(3,:)+a3*[cos(q(1)+q(2)+q(3)) sin(q(1)+q(2)+q(3))];
    pos=p;
end