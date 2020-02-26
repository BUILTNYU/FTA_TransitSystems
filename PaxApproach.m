function [OAvailSeg,DAvailSeg]=PaxApproach(VEH,PAX,ChkPnts,walklimit,walkspeed,t)
% % INPUTS % %
% VEH: vehicle data structure
% PAX: passegner data structure
% ChkPnts: checkpoint information (ID,x,y)
% walkspeed: walking speed
% walkspeed: walking speed
% t: current time

% % OUTPUTS % %
% OAvailSeg: approachable segment from origin
% DAvailSeg: approachable segment from destination

% input vehicle and passenger information
OAvailSeg=[];       % empty for OAvailSeg
DAvailSeg=[];       % empty for DAvailSeg
Sect=zeros(2,2);    % matrix for start and end point of approachable segments
if PAX.drctn==1 % trip is rightward
    RtPrsnt=VEH.Rt1;        % current route
    RtCumCost=VEH.CumCost1; % current cumulative segment travel time
    drctn=1;                % current vehicle direction
else % otherwise (opposite direction)
    RtPrsnt=VEH.Rt2;
    RtCumCost=VEH.CumCost2;
    drctn=2;
end

% identify section between adjacent checkpoints which includes true origin for passenger with NP (type 3, 4)
if PAX.type>=3
    OSign(:,1)=sign(ChkPnts(:,2)-PAX.O(1,2));  % sign of horizontal direction from O to checkpoints
    p=1;    % location index of true origin
    while OSign(p,1)*OSign(p+1,1)<=0    % find p where sign of horizontal direction changes
        p=p+1;              
    end
    Sect(1,1:2)=[p,p+1];    % p: upstream checkpoint, p+1: downstream checkpoint
end

% identify section between adjacent checkpoints which includes true destination for passenger with ND (type 2, 4)
if mod(PAX.type,2)==0 %(type 2,4)
    DSign(:,1)=sign(ChkPnts(:,2)-PAX.D(1,2));  % sign of horizontal direction from D to checkpoints
    q=1;    % location index of true destination
    while DSign(q,1)*DSign(q+1,1)<=0
        q=q+1;          
    end
    Sect(2,1:2)=[q,q+1];    % q: upstream checkpoint, q+1: downstream checkpoint
end
if drctn==2 % make checkpoint sequence consistent with passenger trip direction
    Sect=fliplr(Sect);  
end

% find available segments for true origin for passenger with type 3 or 4
if (PAX.type==3 && sum(RtPrsnt(:,1)==Sect(1,2))>0) || (PAX.type==4 && sum(RtPrsnt(:,1)==Sect(1,2))>0 && sum(RtPrsnt(:,1)==Sect(2,1))>0 && sum(RtPrsnt(:,1)==Sect(2,2))>0)
    % identify section between adjacent checkpoints
    [e1,~]=find(RtPrsnt(:,1)==Sect(1,2));   % sequence of downstream checkpoint
    if sum(RtPrsnt(:,1)==Sect(1,1))>0       % upstream checkpoint is included in current route
        [s1,~]=find(RtPrsnt(:,1)==Sect(1,1));   % sequence of upstream checkpoint
        RtSectO=RtPrsnt(s1:e1,:);               % extract segments between s1-th and e1-th stop from current route
    else  % upstream checkpoint is not included
        RtSectO=[-1,VEH.LocNow,t,0,0,VEH.load;RtPrsnt(1:e1,:)];   % extract segments between current vehicle location and e1-th stop from current route
        RtCumCost=[0;RtCumCost];    % adjust cumulative segment travel cost to consider current vehicle location
    end
    
    % identify legs between stops or turning points (leg: component of segment / segment: stop to stop (stops include checkpoints)/ section: checkpoint to checkpoint)
    k=1;    % index for sequence of leg
    RtSectOSeg=zeros(1,1);      % prepare array for all legs within section
    for i=1:size(RtSectO,1)-1
        if RtSectO(i,2)==RtSectO(i+1,2) || RtSectO(i,3)==RtSectO(i+1,3) % there is no turning point (3 cases within 1 leg)
            RtSectOSeg(k,1:3)=RtSectO(i,1:3);   % Start (stop) x, y, prepare row for additional analysis (start)
            RtSectOSeg(k,4:6)=RtSectO(i+1,1:3); % End (stop) x, y
            RtSectOSeg(k+1,:)=RtSectOSeg(k,:);  % prepare rows for additional analysis (mid)
            RtSectOSeg(k+2,:)=RtSectOSeg(k,:);  % prepare rows for additional analysis (end)
            RtSectOSeg(k:k+2,15)=[-1;0;1];      % case index (-1: start/0: middle/1: end)
            k=k+3;  % increase index for next leg
        else % there is a turning point (2 legs in this segment and 5 cases in total)
            RtSectOSeg(k,1:3)=RtSectO(i,1:3);       % 1st Start (stop) x, y, prepare row for additional analysis (start)
            RtSectOSeg(k,4:6)=[0,RtSectO(i,2),RtSectO(i+1,3)]; % 1st End (turning) x, y
            RtSectOSeg(k+1,:)=RtSectOSeg(k,:);      % prepare rows for additional analysis (mid)
            RtSectOSeg(k+2,:)=RtSectOSeg(k,:);      % prepare rows for additional analysis (end, turning point)
            RtSectOSeg(k+3,1:3)=RtSectOSeg(k,4:6);  % 2nd Start == 1st End x, y, prepare row for additional analysis (mid)
            RtSectOSeg(k+3,4:6)=RtSectO(i+1,1:3);   % 2nd End (stop) x, y
            RtSectOSeg(k+4,:)=RtSectOSeg(k+2,:);    % prepare rows for additional analysis (end)
            RtSectOSeg(k:k+4,15)=[-1;0;1;0;1];      % case index (-1: start/0: middle/1: end)
            k=k+5;  % increase index for next segment
        end
    end
    
    % analyze approaches to legs
    for k=1:size(RtSectOSeg,1)
        if RtSectOSeg(k,15)==-1     % consider start point of leg
            d=GridDist(PAX.O(1,2:3),RtSectOSeg(k,2:3));     % distance to start point
            RtSectOSeg(k,7:11)=[RtSectOSeg(k,1:3),-1,d];    % [start point (ID,x,y), case index and distance]
        elseif RtSectOSeg(k,15)==1  % consider end point of leg
            d=GridDist(PAX.O(1,2:3),RtSectOSeg(k,5:6));     % distance to end point
            RtSectOSeg(k,7:11)=[RtSectOSeg(k,4:6),1,d];     % [end point (ID,x,y), case index and distance]
        else % consider middle point
            if RtSectOSeg(k,2)==RtSectOSeg(k,5)     % vertical leg
                d=abs(PAX.O(1,2)-RtSectOSeg(k,2));  % shortest distance to leg
                RtSectOSeg(k,7:11)=[0,RtSectOSeg(k,2),PAX.O(1,3),0,d];  % [intersection point (null ID,x,y), case index and distance]
            elseif RtSectOSeg(k,3)==RtSectOSeg(k,6) % horizontal leg
                d=abs(PAX.O(1,3)-RtSectOSeg(k,3));  % shortest distance to seg
                RtSectOSeg(k,7:11)=[0,PAX.O(1,2),RtSectOSeg(k,3),0,d];  % [intersection point (null ID,x,y), case index and distance]
            end
        end
    end
    
    % evaluate the feasibility of sending passenger to this point (pax arrival < vehicle arrival or departure)
    RtSectOSeg(:,12)=RtSectOSeg(:,11)/walkspeed*3600;   % add column of access time
    RtSectOSeg=sortrows(RtSectOSeg,11);                 % sort legs by required walking time
    r=1;    % row index
    RtSectOSeg(:,13)=-1;    % assign negative value which will be replaced by temporary wait time
    while r<=size(RtSectOSeg,1)
        if RtSectOSeg(r,11)<=walklimit  % distance to intersection point is shorter than walklimit
            if RtSectOSeg(r,10)==-1 && RtSectOSeg(r,1)~=0   % passenger can be picked up at start point of segment
                tempwaitt=RtCumCost(RtSectO(:,1)==RtSectOSeg(r,1),1)-t-RtSectOSeg(r,12);  % departure time - current time - walking time
            elseif RtSectOSeg(r,10)==1 && RtSectOSeg(r,4)>0 % passenger can be picked up at end point of segment
                tempwaitt=RtCumCost(RtSectO(:,1)==RtSectOSeg(r,4),1)-t-RtSectOSeg(r,12);    % departure time - current time - walking time
            elseif RtSectOSeg(r,10)==1 && RtSectOSeg(r,4)==0    % passenger can be picked up at turning point within segment
                tempwaitt=RtCumCost(RtSectO(:,1)==RtSectOSeg(r,1),1)+GridDist(RtSectOSeg(r,2:3),RtSectOSeg(r,5:6))/VEH.v*3600-t-RtSectOSeg(r,12);   % departure time + running time - current time - walking time
            elseif RtSectOSeg(r,10)==0 % passenger can be picked up at middle point of leg
                if RtSectOSeg(r,2)==RtSectOSeg(r,5) % leg is vertical 
                    if RtSectOSeg(r,1)~=0       % start point is existing one
                        tempwaitt=RtCumCost(RtSectO(:,1)==RtSectOSeg(r,1),1)+abs(RtSectOSeg(r,3)-PAX.O(1,3))/VEH.v*3600-t-RtSectOSeg(r,12); % departure time + running time - current time - walking time
                    elseif RtSectOSeg(r,4)>0    % end point is existing one
                        m1=find(RtSectO(:,1)==RtSectOSeg(r,4)); % find sequence of end point in extracted route
                        tempwaitt=RtCumCost(m1-1,1)+GridDist(RtSectO(m1-1,2:3),[RtSectOSeg(r,5),PAX.O(1,3)])/VEH.v*3600-t-RtSectOSeg(r,12); % departure time + running time - current time - walking time
                    end
                else % leg is horizontal
                    if RtSectOSeg(r,1)~=0       % start point is existing one
                        tempwaitt=RtCumCost(RtSectO(:,1)==RtSectOSeg(r,1),1)+abs(RtSectOSeg(r,2)-PAX.O(1,2))/VEH.v*3600-t-RtSectOSeg(r,12); % departure time + running time - current time - walking time
                    elseif RtSectOSeg(r,4)>0    % end point is existing one
                        m1=find(RtSectO(:,1)==RtSectOSeg(r,4)); % find sequence of end point in extracted route
                        tempwaitt=RtCumCost(m1-1,1)+GridDist(RtSectO(m1-1,2:3),[PAX.O(1,2),RtSectOSeg(r,6)])/VEH.v*3600-t-RtSectOSeg(r,12); % departure time + running time - current time - walking time
                    end
                end
            end
            if tempwaitt>0 % pax wait time
                RtSectOSeg(r,13:14)=[tempwaitt,VEH.ID]; % save temporary wait time and vehicle ID
            end
            r=r+1;  % increase row index
        else
            break
        end
    end
    OAvailSeg=RtSectOSeg(RtSectOSeg(:,13)>0,:); % get rows that shows positive wait time
end

% find available segments for true destination for passenger with type 2 or 4
if (PAX.type==2 && sum(RtPrsnt(:,1)==Sect(2,1))>0 && sum(RtPrsnt(:,1)==Sect(2,2))>0) || (PAX.type==4 && sum(RtPrsnt(:,1)==Sect(1,1))>0 && sum(RtPrsnt(:,1)==Sect(1,2))>0 && sum(RtPrsnt(:,1)==Sect(2,1))>0 && sum(RtPrsnt(:,1)==Sect(2,2))>0)
    % identify section between adjacent checkpoints
    [s2,~]=find(RtPrsnt(:,1)==Sect(2,1));   % sequence of upstream checkpoint
    [e2,~]=find(RtPrsnt(:,1)==Sect(2,2));   % sequence of downstream checkpoint
    RtSectD=RtPrsnt(s2:e2,:);               % extract segments between s1-th and e1-th stop from current route
        
    % identify legs between stops or turning points (leg: component of segment / segment: stop to stop (stops include checkpoints)/ section: checkpoint to checkpoint)
    k=1;    % index for sequence of leg
    RtSectDSeg=zeros(1,1);      % prepare array for all legs within section
    for i=1:size(RtSectD,1)-1
        if RtSectD(i,2)==RtSectD(i+1,2) || RtSectD(i,3)==RtSectD(i+1,3) % there is no turning point (3 cases within 1 leg)
            RtSectDSeg(k,1:3)=RtSectD(i,1:3);   % Start (stop) x, y, prepare row for additional analysis (start)
            RtSectDSeg(k,4:6)=RtSectD(i+1,1:3); % End (stop) x, y
            RtSectDSeg(k+1,:)=RtSectDSeg(k,:);  % prepare rows for additional analysis (mid)
            RtSectDSeg(k+2,:)=RtSectDSeg(k,:);  % prepare rows for additional analysis (end)
            RtSectDSeg(k:k+2,15)=[-1;0;1];      % case index (-1: start/0: middle/1: end)
            k=k+3;  % increase index for next leg
        else % there is a turning point (2 legs in this segment and 5 cases in total)
            RtSectDSeg(k,1:3)=RtSectD(i,1:3);       % 1st Start (stop) x, y, prepare row for additional analysis (start)
            RtSectDSeg(k,4:6)=[0,RtSectD(i,2),RtSectD(i+1,3)]; % 1st End (turning) x, y
            RtSectDSeg(k+1,:)=RtSectDSeg(k,:);      % prepare rows for additional analysis (mid)
            RtSectDSeg(k+2,:)=RtSectDSeg(k,:);      % prepare rows for additional analysis (end, turning point)
            RtSectDSeg(k+3,1:3)=RtSectDSeg(k,4:6);  % 2nd Start == 1st End x, y, prepare row for additional analysis (mid)
            RtSectDSeg(k+3,4:6)=RtSectD(i+1,1:3);   % 2nd End (stop) x, y
            RtSectDSeg(k+4,:)=RtSectDSeg(k+2,:);    % prepare rows for additional analysis (end)
            RtSectDSeg(k:k+4,15)=[-1;0;1;0;1];      % case index (-1: start/0: middle/1: end)
            k=k+5;  % increase index for next segment
        end
    end
    
    % analyze approaches from legs
    for k=1:size(RtSectDSeg,1)
        if RtSectDSeg(k,15)==-1     % consider start point of leg
            d=GridDist(PAX.D(1,2:3),RtSectDSeg(k,2:3));     % distance to start point
            RtSectDSeg(k,7:11)=[RtSectDSeg(k,1:3),-1,d];    % [start point (ID,x,y), case index and distance]
        elseif RtSectDSeg(k,15)==1  % consider end point of leg
            d=GridDist(PAX.D(1,2:3),RtSectDSeg(k,5:6));     % distance to end point
            RtSectDSeg(k,7:11)=[RtSectDSeg(k,4:6),1,d];     % [end point (ID,x,y), case index and distance]
        else % consider middle point
            if RtSectDSeg(k,2)==RtSectDSeg(k,5)     % vertical leg
                d=abs(PAX.D(1,2)-RtSectDSeg(k,2));  % shortest distance to leg
                RtSectDSeg(k,7:11)=[0,RtSectDSeg(k,2),PAX.D(1,3),0,d];  % [intersection point (null ID,x,y), case index and distance]
            elseif RtSectDSeg(k,3)==RtSectDSeg(k,6) % horizontal leg
                d=abs(PAX.D(1,3)-RtSectDSeg(k,3));  % shortest distance to seg
                RtSectDSeg(k,7:11)=[0,PAX.D(1,2),RtSectDSeg(k,3),0,d];  % [intersection point (null ID,x,y), case index and distance]
            end
        end
    end
    
    % evaluate walking distance from this point
    RtSectDSeg(:,12)=RtSectDSeg(:,11)/walkspeed*3600;   % add column of egress time
    RtSectDSeg=sortrows(RtSectDSeg,11);                 % sort legs by required walking time
    r=1;    % row index
    RtSectDSeg(:,14)=VEH.ID;    % assign vehicle ID
    while r<=size(RtSectDSeg,1)
        if RtSectDSeg(r,11)<=walklimit  % distance to intersection point is shorter than walklimit
            r=r+1;
        else
            break
        end
    end
    DAvailSeg=RtSectDSeg(1:r-1,:);  % get rows with walkable distance
end