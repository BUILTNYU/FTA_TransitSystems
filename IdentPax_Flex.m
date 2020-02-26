function [IncomingPax]=IdentPax_Flex(cumNumPax2,cumNumPax1,Pax,expttc,dwellt,ChkPnts,walklimit)
% % INPUTS % %
% cumNumPax2: ID of last new passenger in this time step
% cumNumPax1: ID of first new passenger in this time step
% Pax: Passenger information
% expttc: expected travel time cost ()
% dwellt: average dwell time at stop
% ChkPnts: checkpoint information (ID,x,y)
% walklimit: maximum walking distance (mi)

% % OUTPUTS % %
% IncomingPax: Passenger data structure

IP=zeros(1,size(Pax,2)-1);  % interim array for passenger information
IncomingPax=struct;         % prepare data structure

% process passenger information
for i=cumNumPax1+1:cumNumPax2
    IP(1,:)=Pax(i,2:end);               % import information from Pax
    IncomingPax(i-cumNumPax1).id=i;     % give passenger ID
    
    % identify whether passenger OD is walkable from nearest checkpoint or not and determine passenger type
    C=size(ChkPnts,1);  % number of checkpoints
    ChkDist=zeros(C,2);	% prepare matrix for distance between O/D and checkpoint
    for j=1:C
        ChkDist(j,1)=GridDist(IP(1,2:3),ChkPnts(j,2:3));    % rectilinear distance between origin and j-th checkpoint
        ChkDist(j,2)=GridDist(IP(1,5:6),ChkPnts(j,2:3));    % rectilinear distance between destination and j-th checkpoint
    end
    [md1,mi1]=min(ChkDist(:,1));    % distance and ID of checkpoint that is nearest to origin
    [md2,mi2]=min(ChkDist(:,2));    % distance and ID of checkpoint that is nearest to destination
    if md1<=walklimit && md2<=walklimit && mi1~=mi2     % both are shorter than walking distance limit and checkpoints are different
        IncomingPax(i-cumNumPax1).type=1;   % assign passenger type "PD (regular Pickup/regular Drop-off)"
        IncomingPax(i-cumNumPax1).O=[mi1,IP(1,2:3),i,3,dwellt,1];   % replace O stop ID with checkpoint
        IncomingPax(i-cumNumPax1).D=[mi2,IP(1,5:6),i,3,dwellt,-1];  % replace O stop ID with checkpoint
        if ChkPnts(mi1,2)<ChkPnts(mi2,2)    % pickup checkpoint is on the left of drop-off checkpoint
            IncomingPax(i-cumNumPax1).drctn=1;  % passenger trip direction is rightward
        else
            IncomingPax(i-cumNumPax1).drctn=2;  % passenger trip direction is leftward
        end
    elseif md1<=walklimit && md2>walklimit     % distance between O and checkpoint is shorter than limit
        IncomingPax(i-cumNumPax1).type=2;   % assign passenger type "PND (regular Pickup/non-regular Drop-off)"
        IncomingPax(i-cumNumPax1).O=[mi1,IP(1,2:3),i,3,dwellt,1]; 
        IncomingPax(i-cumNumPax1).D=[2*i+100,IP(1,5:6),i,2,dwellt,-1];
        if ChkPnts(mi1,2)<IP(1,5)   % pickup checkpoint is on the left of drop-off point
            IncomingPax(i-cumNumPax1).drctn=1;
        else
            IncomingPax(i-cumNumPax1).drctn=2;
        end
    elseif md1>walklimit && md2<=walklimit     % distance between D and checkpoint is shorter than limit
        IncomingPax(i-cumNumPax1).type=3;   % assign passenger type "NPD (non-regular Pickup/regular Drop-off)"
        IncomingPax(i-cumNumPax1).O=[2*i+99,IP(1,2:3),i,1,dwellt,1]; 
        IncomingPax(i-cumNumPax1).D=[mi2,IP(1,5:6),i,3,dwellt,-1];
        if IP(1,2)<ChkPnts(mi2,2)   % pickup point is on the left of drop-off checkpoint
            IncomingPax(i-cumNumPax1).drctn=1;
        else
            IncomingPax(i-cumNumPax1).drctn=2;
        end
    else     % none of them are close enough
        IncomingPax(i-cumNumPax1).type=4;   % assign passenger type "NPND (non-regular Pickup/non-regular Drop-off)"
        IncomingPax(i-cumNumPax1).O=[2*i+99,IP(1,2:3),i,1,dwellt,1];
        IncomingPax(i-cumNumPax1).D=[2*i+100,IP(1,5:6),i,2,dwellt,-1];
        if IP(1,2)<IP(1,5)  % pickup point is on the left of drop-off point
            IncomingPax(i-cumNumPax1).drctn=1;
        else
            IncomingPax(i-cumNumPax1).drctn=2;
        end
    end
    
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
    IncomingPax(i-cumNumPax1).OO=[0,0,0,0,0,0];     % Space for original origin
    IncomingPax(i-cumNumPax1).DD=[0,0,0,0,0,0];     % Space for original destination
    IncomingPax(i-cumNumPax1).LocLog=[0,0];         % Location log
    IncomingPax(i-cumNumPax1).AddWalkT=[0,0];       % Additional walking time (owalk for Type 3&4, dwalk for Type 2&4)
end