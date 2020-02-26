function [IncomingPax]=IdentPax_Fix(cumNumPax2,cumNumPax1,Pax,expttc,dwellt,Stops)
% % INPUTS % %
% cumNumPax2: ID of last new passenger in this time step
% cumNumPax1: ID of first new passenger in this time step
% Pax: Passenger information
% expttc: expected travel time cost (inverse of average vehicle speed)
% dwellt: average dwell time at stop
% Stops: fixed stop information (ID,x,y)

% % OUTPUTS % %
% IncomingPax: Passenger data structure

IP=zeros(1,size(Pax,2)-1);  % interim array for passenger information
IncomingPax=struct;         % prepare data structure

% process passenger information
for i=cumNumPax1+1:cumNumPax2
    IP(1,:)=Pax(i,2:end);           % import information from Pax
    IncomingPax(i-cumNumPax1).id=i; % give passenger ID
    
    % recognize the nearest stops from passenger OD
    C=size(Stops,1);    % number of fixed stops
    DistC=zeros(C,2);   % matrix for distances between stops and passenger O or D
    for j=1:C
        DistC(j,1)=GridDist(IP(1,2:3),Stops(j,2:3));    % distance between j-th stop and O
        DistC(j,2)=GridDist(IP(1,5:6),Stops(j,2:3));    % distance between j-th stop and D
    end
    
    % designate nearest stops
    [~,mi1]=min(DistC(:,1));  % find nearest stops from O
    [~,mi2]=min(DistC(:,2));  % find nearest stops from D
    
    IP(1,1)=mi1;
    IP(1,4)=mi2;
    IncomingPax(i-cumNumPax1).type=1;
    IncomingPax(i-cumNumPax1).O=[IP(1,1:3),i,3,dwellt,1];
    IncomingPax(i-cumNumPax1).D=[IP(1,4:6),i,3,dwellt,-1];
    
    IncomingPax(i-cumNumPax1).hDist=IP(1,7);        % horizontal distance
    IncomingPax(i-cumNumPax1).vDist=IP(1,8);        % vertical distance
    IncomingPax(i-cumNumPax1).onbrd=0;              % rejection indicator
    IncomingPax(i-cumNumPax1).drtt=expttc*(IP(1,7)+IP(1,8));    % expected direct travel time between OD
    IncomingPax(i-cumNumPax1).extt=0;               % expected in-vehicle time
    IncomingPax(i-cumNumPax1).rtt=0;                % in-vehicle time in real time
    IncomingPax(i-cumNumPax1).exwt=0;               % expected wait time
    IncomingPax(i-cumNumPax1).rwt=0;                % wait time in real time
    IncomingPax(i-cumNumPax1).btime=0;              % time step when pax boards
    IncomingPax(i-cumNumPax1).atime=0;              % time step when pax alights
    IncomingPax(i-cumNumPax1).veh=0;                % vehicle ID of which passenger is assigned
end