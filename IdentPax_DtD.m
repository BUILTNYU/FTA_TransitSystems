function [IncomingPax]=IdentPax_DtD(cumNumPax2,cumNumPax1,Pax,expttc,dwellt)
% % INPUTS % %
% cumNumPax2: ID of last new passenger in this time step
% cumNumPax1: ID of first new passenger in this time step
% Pax: Passenger information
% expttc: expected travel time cost (inverse of average vehicle speed)
% dwellt: average dwell time at stop

% % OUTPUTS % %
% IncomingPax: Passenger data structure

IP=zeros(1,size(Pax,2)-1);  % interim array for passenger information
IncomingPax=struct;         % prepare data structure

% process passenger information
for i=cumNumPax1+1:cumNumPax2
    IP(1,:)=Pax(i,2:end);               % import information from Pax
    IncomingPax(i-cumNumPax1).id=i;     % give passenger ID
%     IncomingPax(i-cumNumPax1).type=0;   % 
    
    % O,D: [StopID,X,Y,StopType,PaxID,dwellt,numPax]
    IncomingPax(i-cumNumPax1).O=[2*i+99,IP(1,2:3),i,1,dwellt,1];
    IncomingPax(i-cumNumPax1).D=[2*i+100,IP(1,5:6),i,2,dwellt,-1];
    
    IncomingPax(i-cumNumPax1).hDist=IP(1,7);        % horizontal distance
    IncomingPax(i-cumNumPax1).vDist=IP(1,8);        % vertical distance
    IncomingPax(i-cumNumPax1).onbrd=0;              % rejection indicator
    IncomingPax(i-cumNumPax1).drtt=expttc*(IP(1,7)+IP(1,8)); % expected direct travel time between OD
    IncomingPax(i-cumNumPax1).extt=0;               % expected in-vehicle time
    IncomingPax(i-cumNumPax1).rtt=0;                % in-vehicle time in real time
    IncomingPax(i-cumNumPax1).exwt=0;               % expected wait time
    IncomingPax(i-cumNumPax1).rwt=0;                % wait time in real time
    IncomingPax(i-cumNumPax1).btime=0;              % time step when pax boards
    IncomingPax(i-cumNumPax1).atime=0;              % time step when pax alights
    IncomingPax(i-cumNumPax1).veh=0;                % vehicle ID of which passenger is assigned
end