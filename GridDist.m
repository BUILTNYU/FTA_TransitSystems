function [s]=GridDist(A,B)
% calculate sum of horizontal and vertical distance between A and B
s=abs(A(1,1)-B(1,1))+abs(A(1,2)-B(1,2));