function [Pax,numNewPax]=PaxGen(HRange,VRange,walklimit,avgReq,nstops)
% INPUT
% HRange: horizontal range of service area
% VRange: vertical range of service area
% walklimit: maximum distance for walking (mi)
% avgReq: average number of requests received
% nstops: number of stops on fixed route

% OUTPUT
% Pax: Passenger pools with individual information
    % Column1: Passenger ID, given in a "First-come-first-served" scheme
    % Column2: Stop ID of passenger's origin (empty, assigned during simulation)
    % Column3,4: X,Y of origin
    % Column5: Stop ID of passenger's destination (empty, assigned during simulation)
    % Column6,7: X,Y of destination
    % Column8,9: horizontal/vertical distance between OD

% calculate total required number of passengers
nPax=avgReq*3600*30;    % generate sufficient passengers (e.g. equivalent to 30 hours)

% prepare pax pools
Pax = [(1:nPax)',zeros(nPax,8)];    % Column1: ID/Others: empty

% generate random x-y coordinates
k=1;    % row index
while k<=nPax % repeat until k reaches nPax
    while sum(Pax(k,8:9))<walklimit || abs(Pax(k,6)-Pax(k,3))<=((HRange(1,2)-HRange(1,1))/(nstops-1))
    % generated passenter should meet conditions:
    % 1. Walking distance between OD should not be shorter than walklimit
    % 2. Horizontal distance should be longer than a spacing between two adjacent fixed stops
        Pax(k,2)=0;                                             % OStopID
        Pax(k,3)=rand*(HRange(1,2)-HRange(1,1));                % O x
        Pax(k,4)=rand*(VRange(1,2)-VRange(1,1))+VRange(1,1);    % O y
        Pax(k,5)=0;                                             % DStopID
        Pax(k,6)=rand*(HRange(1,2)-HRange(1,1));                % D x
        Pax(k,7)=rand*(VRange(1,2)-VRange(1,1))+VRange(1,1);    % D y
        Pax(k,8)=abs(Pax(k,6)-Pax(k,3));                % horizontal distance
        Pax(k,9)=abs(Pax(k,7)-Pax(k,4));                % vertical distance
    end
    k=k+1; % move on to the next passenger
end

% generate number of passenger per time step
numNewPax=zeros(100000,1);       % number of new passenger per time step
for i=1:100000
    numNewPax(i,1)=poissinv(rand,avgReq);   % number of new passengers assumed to follow Poisson distribution
end