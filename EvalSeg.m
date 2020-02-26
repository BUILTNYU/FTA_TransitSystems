function [CAND]=EvalSeg(VEH,PAX,alpha,beta,C,t,OAvailSeg,DAvailSeg,walkspeed,maxwait)
% % INPUTS % %
% VEH: vehicle data structure
% PAX: passegner data structure
% alpha: weight for in-vehicle time
% beta: weight for wait time
% C: number of checkpoint
% t: current time
% OAvailSeg: approachable segment from origin
% DAvailSeg: approachable segment from destination
% walkspeed: walking speed
% maxwait: maximum wait time

% % OUTPUTS % %
% CAND: candidate route data structure


% input vehicle and passenger information
CAND=struct;    % prepare data structure for candidate route
if PAX.drctn==1 % passenger trip is rightward
    RtPrsnt=VEH.Rt1;    % current route
    RtCost=VEH.Cost1;   % current segment travel time
    ST=VEH.SlackT1;     % current slack time
    drctn=1;            % current vehicle direction
    Pax=VEH.Pax1;       % currently involved passenger
    WalkT=VEH.WalkT1;   % walking time of currently involved passenger
else % trip is leftward
    RtPrsnt=VEH.Rt2;
    RtCost=VEH.Cost2;
    ST=VEH.SlackT2;
    drctn=2;
    Pax=VEH.Pax2;
    WalkT=VEH.WalkT2;
end

% consider inserting new passenger between current vehicle location and 1st stop
if VEH.LocNow(1,1)~=RtPrsnt(1,2) && VEH.LocNow(1,2)~=RtPrsnt(1,3) && VEH.drctn==PAX.drctn % vehicle did not arrive at the next stop, and new passenger can be added before 1st stop
    veh1st=1;   % index of initial vehicle location consideration (0: not included,1:included)
    RtPrsnt=[-1,VEH.LocNow,t,0,0,VEH.load;RtPrsnt];  % consider current vehicle loaction as 1st stop of route
    RtCost=[0;RtCost];      % add zero for segment travel cost to current vehicle location
else
    veh1st=0;   % index of initial vehicle location consideration (0: not included,1:included)
end

numPaxb=size(Pax,1);	% number of existing pax

% create RtCumCost: cumulative travel cost from current time to departure time of each stop
if veh1st==1 % current vehicle location is included in RtPrsnt
    if RtPrsnt(2,5)==3	% 1st stop is checkpoint
        RtCumCost(2,1)=RtPrsnt(2,4)-t;	% predetermined departure time - current time
    else % normal stop
        RtCumCost(2,1)=RtCost(2,1)+RtPrsnt(2,6);	% travel time from current location to 1st stop
    end
    for i=3:size(RtCost,1)
        RtCumCost(i,1)=RtCumCost(i-1,1)+RtCost(i,1);    % accumulate cost (cumulative segment travel time to previous stop + segment travel time to i-th stop + dwell time
        if RtPrsnt(i,5)==3	% i-th stop is checkpoint
            RtCumCost(i,1)=RtPrsnt(i,4)-t;	% predetermined departure time - current time
        end
    end
else % not included
    if RtPrsnt(1,5)==3  % 1st stop is checkpoint
        RtCumCost(1,1)=RtPrsnt(1,4)-t;  % predetermined departure time - current time
    else % normal stop
        RtCumCost(1,1)=RtCost(1,1)+RtPrsnt(1,6);    % travel time from current location to 1st stop
    end
    for i=2:size(RtCost,1)
        RtCumCost(i,1)=RtCumCost(i-1,1)+RtCost(i,1);    % accumulate cost (cumulative segment travel time to previous stop + segment travel time to i-th stop + dwell time
        if RtPrsnt(i,5)==3  % i-th stop is checkpoint
            RtCumCost(i,1)=RtPrsnt(i,4)-t;  % predetermined departure time - current time
        end
    end
end

% performance measure before adding new customer
EXWTb=zeros(numPaxb,2); % expected wait time for existing passenger
EXTTb=zeros(numPaxb,2); % expected in-vehicle time for existing passenger
for m=2:numPaxb
    if Pax(m,4)==0  % passenger is not picked up yet
        EXWTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,2),1)]; % EXWT Step 1. CumCost to origin
        if RtPrsnt(RtPrsnt(:,1)==Pax(m,2),5)==3 % origin is checkpoint
            if drctn==1 % direction is rightward
                if VEH.drctn==2 && Pax(m,2)==1          % vehicle is opposite direction and origin is a checkpoint #1
                    EXWTb(m,2)=EXWTb(m,2)-VEH.SlackT2(C-1,2);   % EXWT Step 2-1. CumCost to origin - last slack time in opposite direction
                elseif VEH.drctn==2 && Pax(m,2)~=1      % origin is a checkpoint other than #1
                    EXWTb(m,2)=EXWTb(m,2)-ST(Pax(m,2)-1,2);     % EXWT Step 2-2. CumCost to origin - slack time in same direction
                elseif VEH.drctn==1 && EXWTb(m,2)==0    % vehicle is rightward and expected wait time is zero (passenger is onboard but waiting for departure)
                    EXWTb(m,2)=0;                               % EXWT Step 2-3. expected wait time remains zero
                elseif VEH.drctn==1 && Pax(m,2)~=1      % vehicle is rightward and origin is checkpoint other than #1
                    EXWTb(m,2)=EXWTb(m,2)-ST(Pax(m,2)-1,2);     % EXWT Step 2-4. CumCost to origin - slack time of that checkpoint
                end
            elseif drctn==2 % direction is leftward
                if VEH.drctn==1 && Pax(m,2)==C
                    EXWTb(m,2)=EXWTb(m,2)-VEH.SlackT1(C-1,2);
                elseif VEH.drctn==1 && Pax(m,2)~=C
                    EXWTb(m,2)=EXWTb(m,2)-ST(C-Pax(m,2),2);
                elseif VEH.drctn==2 && EXWTb(m,2)==0
                    EXWTb(m,2)=0;
                elseif VEH.drctn==2 && Pax(m,2)~=C
                    EXWTb(m,2)=EXWTb(m,2)-ST(C-Pax(m,2),2);
                end
            end
        else % origin is not checkpoint
            EXWTb(m,2)=EXWTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,2),6);
        end
        EXTTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,3),1)-EXWTb(m,2)]; % EXTT Step 1. CumCost to destination - EXWT
        EXWTb(m,2)=max(0,EXWTb(m,2)-WalkT(WalkT(:,1)==Pax(m,1),2));             % EXWT Step 3. deduct walking time from expected wait time

        if RtPrsnt(RtPrsnt(:,1)==Pax(m,3),5)==3 % destination is checkpoint
            if drctn==1 % direction is rightward
                EXTTb(m,2)=EXTTb(m,2)-ST(Pax(m,3)-1,2); % EXTT Step 2-1. EXTT - slack time
            elseif drctn==2 % direction is leftward
                EXTTb(m,2)=EXTTb(m,2)-ST(C-Pax(m,3),2); % EXTT Step 2-1. EXTT - slack time
            end
        else % destination is not checkpoint
            EXTTb(m,2)=EXTTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,3),6);    % EXTT Step 2-2. EXTT - dwell time
        end
    elseif Pax(m,4)==1	% if pax is picked up previously and onboard
        EXTTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,3),1)]; % EXTT Step 1. CumCost to destination
        if RtPrsnt(RtPrsnt(:,1)==Pax(m,3),5)==3 % destination is chkpnt
            if drctn==1 % direction is rightward
                EXTTb(m,2)=EXTTb(m,2)-ST(Pax(m,3)-1,2); % EXTT Step 2-1. EXTT - slack time
            elseif drctn==2 % direction is leftward
                EXTTb(m,2)=EXTTb(m,2)-ST(C-Pax(m,3),2); % EXTT Step 2-1. EXTT - slack time
            end
        else % destination is not checkpoint
            EXTTb(m,2)=EXTTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,3),6);    % EXTT Step 2-2. EXTT - dwell time
        end
    end
end
pfmcmsrb=alpha*sum(EXTTb(:,2))+beta*sum(EXWTb(:,2)); % performance measure calculation (linear combination of wait and in-vehicle time)

% insert temporary O stop and/or D stop
% 0.add pax info
PaxInfo=[Pax;PAX.id,PAX.O(1,1),PAX.D(1,1),0]; % existing and new passenger information [PaxID,OStopID,DStopID,Boarding index]
if PAX.type==2      % P-ND
    if DAvailSeg(1,7)>0 % approachable point to destination already exists on route
        PaxInfo(end,3)=DAvailSeg(1,7);  % substitute D Stop ID with this point
    end
elseif PAX.type==3  % N-PD
    if OAvailSeg(1,7)>0 % approachable point from origin already exists on route
        PaxInfo(end,2)=OAvailSeg(1,7);  % substitute O Stop ID with this point
    end
elseif PAX.type==4  % NP-ND
    if OAvailSeg(1,7)>0 % approachable point from origin already exists on route
        PaxInfo(end,2)=OAvailSeg(1,7);  % substitute O Stop ID with this point
    end
    if DAvailSeg(1,7)>0 % approachable point to destination already exists on route
        PaxInfo(end,3)=DAvailSeg(1,7);	% substitute D Stop ID with this point
    end
end

numPax=size(PaxInfo,1);         % number of passenger (including 1st row with zeros)
nRtChk=sum(RtPrsnt(:,5)==3);    % remaining number of checkpoint to visit in this direction

% examine required costs between checkpoints
TCost=zeros(size(RtPrsnt,1),2); % actual cost required to reach next points (travel time + dwell time) [actual cost,section belonged]
if veh1st==1 % current vehicle location is included in RtPrsnt
    if drctn==1 % direction is rightward
        if RtPrsnt(2,1)==1  % 1st stop is checkpoint #1
            TCost(1,2)=0;   % this segment is excluded from calculating section travel time
        else % 1st stop is not checkpoint
            TCost(1,2)=C-nRtChk;    % assign checkpoint ID which this segment can be included
        end
        for i=2:size(RtPrsnt,1)
            TCost(i,1) = 1/VEH.v*3600*GridDist(RtPrsnt(i-1,2:3),RtPrsnt(i,2:3))+RtPrsnt(i,6);   % segment travel time + dwell time
            if RtPrsnt(i-1,5)==3    % (i-1)-th stop is checkpoint
                TCost(i,2)=TCost(i-1,2)+1;  % increase checkpoint ID by 1 as vehicle passes one
            else % (i-1)-th stop is not checkpoint
                TCost(i,2)=TCost(i-1,2);    % keep checkpoint ID
            end
        end
    else % direction is leftward
        if RtPrsnt(2,1)==C	% 1st stop is checkpoint #C
            TCost(1,2)=C;   % this segment is excluded from calculating section travel time
        else % 1st stop is not checkpoint
            TCost(1,2)=nRtChk;  % assign checkpoint ID which this segment can be included
        end
        for i=2:size(RtPrsnt,1)
            TCost(i,1) = 1/VEH.v*3600*GridDist(RtPrsnt(i-1,2:3),RtPrsnt(i,2:3))+RtPrsnt(i,6);   % segment travel time + dwell time
            if RtPrsnt(i-1,5)==3    % (i-1)-th stop is checkpoint
                TCost(i,2)=TCost(i-1,2)-1;  % decrease checkpoint ID by 1 as vehicle passes one
            else % (i-1)-th stop is not checkpoint
                TCost(i,2)=TCost(i-1,2);    % keep checkpoint ID
            end
        end
    end
else % not included
    if drctn==VEH.drctn % vehicle direction is the same as passenger
        TCost(1,1) = 1/VEH.v*3600*GridDist(VEH.LocNow,RtPrsnt(1,2:3))+RtPrsnt(1,6);   % segment travel time from current vehicle location to 1st stop + dwell time
        VehLoad = VEH.load;	% initial vehicle load
    else % vehicle direction is different
        TCost(1,1) = RtPrsnt(1,4)-t;    % departure time - current time
        VehLoad = 0;        % no passenger is onboard
    end
    if drctn==1 % vehicle is rightward
        TCost(1,2)=C-nRtChk;    % this segment belongs to (C-nRtChk)-th section
        for i=2:size(RtPrsnt,1)
            TCost(i,1) = 1/VEH.v*3600*GridDist(RtPrsnt(i-1,2:3),RtPrsnt(i,2:3))+RtPrsnt(i,6);	% segment travel time + dwell time
            if RtPrsnt(i-1,5)==3    % (i-1)-th stop is checkpoint
                TCost(i,2)=TCost(i-1,2)+1;  % increase checkpoint ID by 1 as vehicle passes one
            else % (i-1)-th stop is not checkpoint
                TCost(i,2)=TCost(i-1,2);    % keep checkpoint ID
            end
        end
    else % vehicle is leftward
        TCost(1,2)=nRtChk;  % same as number of remaining checkpoint
        for i=2:size(RtPrsnt,1)
            TCost(i,1) = 1/VEH.v*3600*GridDist(RtPrsnt(i-1,2:3),RtPrsnt(i,2:3))+RtPrsnt(i,6);   % segment travel time + dwell time
            if RtPrsnt(i-1,5)==3    % (i-1)-th stop is checkpoint
                TCost(i,2)=TCost(i-1,2)-1;  % decrease checkpoint ID by 1 as vehicle passes one
            else % (i-1)-th stop is not checkpoint
                TCost(i,2)=TCost(i-1,2);    % keep checkpoint ID
            end
        end
    end
end

% aggregate TCost by sections between checkpoints 
ChkCost=zeros(C-1,4); % prepare matrix for section travel time [currently required travel time,required travel time after insertion,available slack time,backtracking distance]
for i=1:C-1
    if drctn==1 % direction is rightward
        ChkCost(i,1)=sum(TCost(TCost(:,2)==i,1)); % initially required travel time for section before i-th checkpoint
    elseif drctn==2 % leftward
        ChkCost(i,1)=sum(TCost(TCost(:,2)==C-i,1));
    end
end

% 1.change route (add NP and/or ND and update pax change)
m1=0;   % sequence of potential O stop
m2=0;   % sequence of potential D stop
if PAX.type==2 && sum(RtPrsnt(:,1)==PAX.O(1,1)) % P-ND
    [m1,~]=find(RtPrsnt(:,1)==PAX.O(1,1));  % find O checkpoint
    RtPrsnt(m1,7)=RtPrsnt(m1,7)+1;          % load passenger to O Checkpoint
    if DAvailSeg(1,7)>0 % approachable point to destination already exists on route
        [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,7));  % find D stop
        RtPrsnt(m2,7)=RtPrsnt(m2,7)-1;  % unload passenger from m2-th stop
    else % approachable point will be newly inserted point
        if DAvailSeg(1,1)>0 % start point of segment is existing stop
            [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,1));    % find start point
            RtPrsnt=[RtPrsnt(1:m2,:);PAX.D(1,1),DAvailSeg(1,8:9),PAX.id,2,20,-1;RtPrsnt(m2+1:end,:)];   % insert potential D stop to RtPrsnt
        else % end point of segment is existing stop
            [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,4));    % find end point
            RtPrsnt=[RtPrsnt(1:m2-1,:);PAX.D(1,1),DAvailSeg(1,8:9),PAX.id,2,20,-1;RtPrsnt(m2:end,:)];   % insert potential D stop to RtPrsnt
        end
    end
elseif PAX.type==3 && sum(RtPrsnt(:,1)==PAX.D(1,1)) % NP-D
    if OAvailSeg(1,7)>0 % approachable point from origin already exists on route
        [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,7));  % find O stop
        RtPrsnt(m1,7)=RtPrsnt(m1,7)+1;              % load passenger to m1-th stop
    else % approachable point will be newly inserted point
        if OAvailSeg(1,1)~=0 % start point of segment is existing stop or current vehicle location
            [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,1));	% find start point
            RtPrsnt=[RtPrsnt(1:m1,:);PAX.O(1,1),OAvailSeg(1,8:9),PAX.id,1,20,1;RtPrsnt(m1+1:end,:)];   % insert potential O stop to RtPrsnt
        else  % end point of segment is existing stop or current vehicle location
            [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,4));  % find end point
            RtPrsnt=[RtPrsnt(1:m1-1,:);PAX.O(1,1),OAvailSeg(1,8:9),PAX.id,1,20,1;RtPrsnt(m1:end,:)];   % insert potential O stop to RtPrsnt
        end
    end
    [m2,~]=find(RtPrsnt(:,1)==PAX.D(1,1));	% find D checkpoint
    RtPrsnt(m2,7)=RtPrsnt(m2,7)-1;          % unload passenger from m2-th stop
elseif PAX.type==4 % NP-ND
    if OAvailSeg(1,7)>0 % approachable point from origin already exists on route
        [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,7));	% find O stop
        RtPrsnt(m1,7)=RtPrsnt(m1,7)+1;              % load passenger to m1-th stop
    elseif OAvailSeg(1,7)==0
        if OAvailSeg(1,1)~=0 % start point of segment is existing stop or current vehicle location
            [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,1));  % find start point
            RtPrsnt=[RtPrsnt(1:m1,:);PAX.O(1,1),OAvailSeg(1,8:9),PAX.id,1,20,1;RtPrsnt(m1+1:end,:)];	% insert potential O stop to RtPrsnt
        else  % end point of segment is existing stop or current vehicle location
            [m1,~]=find(RtPrsnt(:,1)==OAvailSeg(1,4));  % find end point
            RtPrsnt=[RtPrsnt(1:m1-1,:);PAX.O(1,1),OAvailSeg(1,8:9),PAX.id,1,20,1;RtPrsnt(m1:end,:)];    % insert potential O stop to RtPrsnt
        end
    end
    if DAvailSeg(1,7)>0 % approachable point to destination already exists on route
        [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,7));	% find D stop
        RtPrsnt(m2,7)=RtPrsnt(m2,7)-1;              % unload passenger from m2-th stop
    elseif DAvailSeg(1,7)==0
        if DAvailSeg(1,1)>0 % start point of segment is existing stop or current vehicle location
            [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,1));  % find start point
            RtPrsnt=[RtPrsnt(1:m2,:);PAX.D(1,1),DAvailSeg(1,8:9),PAX.id,1,20,-1;RtPrsnt(m2+1:end,:)];	% insert potential D stop to RtPrsnt
        else % end point of segment is existing stop or current vehicle location
            [m2,~]=find(RtPrsnt(:,1)==DAvailSeg(1,4));  % find end point
            RtPrsnt=[RtPrsnt(1:m2-1,:);PAX.D(1,1),DAvailSeg(1,8:9),PAX.id,1,20,-1;RtPrsnt(m2:end,:)];	% insert potential D stop to RtPrsnt
        end
    end
end

if m1*m2~=0 && m1<m2 % potential O and D stops exist in route and properly ordered
    % labeling stops
    rt=size(RtPrsnt,1);         % length of augmented route
    RtInfo=[(1:rt)',RtPrsnt];   % route information (sequence, route)
    
    % veihcle load profile
    if veh1st==1 % current vehicle location is included in RtPrsnt
        Bload(1,1)=RtInfo(1,8); % initiate current vehicle load profile
    else % not included
        Bload(1,1)=VehLoad+RtInfo(1,8);
    end
    for i=2:rt
        Bload(i,1)=Bload(i-1,1)+RtInfo(i,8);    % accumulate load profile
    end
    
    BST=ST;         % slack time
    mincost = inf;  % assign infinity to route cost

    % reflect addition of new passenger if vehicle capacity is not exceeded
    if max(Bload) <= VEH.q % maximum load does not exceed
        TempCost=zeros(rt,3);       % prepare matrix for required time for segments [required travel and dwell time,section belonged,backtracking distance]
        TempCumCost=zeros(rt,1);    % prepare array for cumulative travel time 
        if veh1st~=1 % current vehicle location is not included in RtPrsnt
            TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(1,3:4))+RtInfo(1,7);  % segment travel time + dwell time
            if RtInfo(1,6)==3   % 1st stop is checkpoint
                TempCumCost(1,1)=RtInfo(1,5)-t; % departure time - current time
            else % not checkpoint
                TempCumCost(1,1)=TempCost(1,1); % same as travel time of 1st segment
            end
        end
        S=zeros(1,C);   % feasibility indicator array

        % initiate Column 2&3 of TempCost
        if drctn==1 % rightward
            TempCost(1,2)=C-nRtChk+1;   % checkpoint ID that involves 1st stop in its section
            TempCost(1,3)=min(0,RtInfo(1,3)-VEH.LocNow(1,1));  % backtracking distance (difference between current location and first stop)
        else % leftward
            TempCost(1,2)=nRtChk;   % checkpoint ID that involves 1st stop in its section
            TempCost(1,3)=min(0,VEH.LocNow(1,1)-RtInfo(1,3));  % backtracking distance (difference between current location and first stop)
        end

        % build temporary segment travel time and cumulative segment travel time
        for m = 1:rt-1
            TempCost(m+1,1)=1/VEH.v*3600*GridDist(RtInfo(m,3:4),RtInfo(m+1,3:4))+RtInfo(m+1,7); % travel time + dwell time
            if drctn==1 % rightward
                TempCost(m+1,3)=min(0,RtInfo(m+1,3)-RtInfo(m,3));   % backtracking distance
            else % leftward
                TempCost(m+1,3)=min(0,RtInfo(m,3)-RtInfo(m+1,3));   % backtracking distance
            end
            if RtInfo(m,6)==3   % m-th stop is checkpoint
                if drctn==1 % rightward
                    TempCost(m+1,2)=TempCost(m,2)+1;	% increase as vehicle passed chkpnt
                else % leftward
                    TempCost(m+1,2)=TempCost(m,2)-1;    % decrease as vehicle passed chkpnt
                end
            else
                TempCost(m+1,2)=TempCost(m,2);  % no change
            end

            % update TempCumCost and check validity of chkpnt arrival and departure
            TempCumCost(m+1,1)=TempCumCost(m,1)+TempCost(m+1,1);        % temporary cumulative segment travel time to previous stop + segment travel time + dwell time
            if RtInfo(m+1,6)==3 && TempCumCost(m+1,1)<=RtInfo(m+1,5)-t  % vehicle can arrive at checkpoint and finish dwelling before departure time
                TempCumCost(m+1,1)=RtInfo(m+1,5)-t;	% replace temporary cumulative segment travel time with departure time - current time
            elseif RtInfo(m+1,6)==3 && TempCumCost(m+1,1)>RtInfo(m+1,5)-t % vehicle cannot
                S=ones(1,C);    % infeasible
                break
            end
        end

        % feasibility check: slack time of sections
        if sum(S)==0	% vehicle can comply timetable
            % check slack times for each section between checkpoints
            CP=RtInfo(RtInfo(:,6)==3,[2,1]);	%[checkpoint ID, route ID)
            for m=1:nRtChk
                if m==1  % m-th checkpoint is 1st checkpoint vehicle visits
                    CP(m,3)=TempCumCost(CP(m,2),1);  % pure cumulative segment travel time
                else
                    CP(m,3)=TempCumCost(CP(m,2),1)-TempCumCost(CP(m-1,2),1);	% cumulative segment travel time from which that of previous checkpoint is deducted
                end
            end
            if size(CP,1)==C	% all checkpoints are on route
                for m=1:C-1
                    ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1,1),1)); % sum of all costs within section
                    ChkCost(m,3)=CP(m+1,3)-RtInfo(CP(m+1,2),7);             % available slack time
                    ChkCost(m,4)=sum(TempCost(TempCost(:,2)==CP(m+1,1),3)); % backtracking distance
                    if ChkCost(m,2)>ChkCost(m,3) || ChkCost(m,2)-ChkCost(m,1)>BST(m,3) || BST(m,5)+ChkCost(m,4)<0
                        % required time is longer than available slack time OR increasing time is longer than usable slack time OR backtracking distance is longer than remaining amount
                        S=ones(1,C);    % infeasible
                        break
                    else % satisfy slack time condition
                        BST(m,2)=BST(m,2)-(ChkCost(m,2)-ChkCost(m,1));	% update available slack time
                        BST(m,5)=BST(m,5)+ChkCost(m,4);                 % update available backtracking
                        if BST(m,2)<BST(m,3) % available slack time is shorter than usable slack time
                            BST(m,3)=BST(m,2);  % cap usable slack time with available slack time
                        end
                    end
                end
            else % some checkpoints are on route
                for m=C-nRtChk:C-1
                    ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1-C+nRtChk,1),1));
                    ChkCost(m,3)=CP(m+1-C+nRtChk,3)-RtInfo(CP(m+1-C+nRtChk,2),7);
                    ChkCost(m,4)=sum(TempCost(TempCost(:,2)==CP(m+1-C+nRtChk,1),3));
                    if ChkCost(m,2)>ChkCost(m,3) || ChkCost(m,2)-ChkCost(m,1)>BST(m,3) || BST(m,5)+ChkCost(m,4)<0
                        S=ones(1,C);
                        break
                    else
                        BST(m,2)=BST(m,2)-(ChkCost(m,2)-ChkCost(m,1));
                        BST(m,5)=BST(m,5)+ChkCost(m,4);
                        if BST(m,2)<BST(m,3)
                            BST(m,3)=BST(m,2);
                        end
                    end
                end
            end
        end

        % check if the expected wait time is longer than threshold
        % walk time calculation
        if PAX.type==2      % P-ND
            owalkt=1/walkspeed*3600*GridDist(PAX.D(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.O(1,1),2:3));   % walking time from true origin to O checkpoint
            dwalkt=DAvailSeg(1,12); % walking time from approachable point to destination
        elseif PAX.type==3  % NP-D
            owalkt=OAvailSeg(1,12); % walking time from true origin to approachable point 
            dwalkt=1/walkspeed*3600*GridDist(PAX.D(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.D(1,1),2:3));   % walking time from D checkpoint to true destination
        else
            owalkt=OAvailSeg(1,12); % walking time from true origin to approachable point 
            dwalkt=DAvailSeg(1,12); % walking time from approachable point to destination
        end

        EXWTc=zeros(numPax,2);  % space for expected wait time
        EXTTc=zeros(numPax,2);  % space for expected in-vehicle time
        WalkTc=[WalkT;PAX.id,owalkt,dwalkt,0,0];	% walking time and other time elements
        for m=2:numPax
            if PaxInfo(m,4)==0 % pax isn't served yet
                EXWTc(m,1:2)=[PaxInfo(m,1),TempCumCost(RtPrsnt(:,1)==PaxInfo(m,2),1)];  % add unserved pax to EXWT
                if RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,2),5)==3 % if origin of pax is chkpnt
                    if drctn==1 % pax is eastbound
                        if VEH.drctn==2 && PaxInfo(m,2)==1  % veh is westbound and origin is ChkPnt#1
                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT2(C-1,2);
                        elseif VEH.drctn==2 && PaxInfo(m,2)~=1  % veh is westbound and origin is the other chkpnt
                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                        elseif VEH.drctn==1 && EXWTc(m,2)==0    % veh is eastbound and cum.cost to origin of pax=1 
                            EXWTc(m,2)=0;
                        elseif VEH.drctn==1 && PaxInfo(m,2)~=1  % veh is eastbound and origin is the other chkpnt
                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                        end
                    elseif drctn==2
                        if VEH.drctn==1 && PaxInfo(m,2)==C  % veh is eastbound and origin is ChkPnt#C
                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT1(C-1,2);
                        elseif VEH.drctn==1 && PaxInfo(m,2)~=C  % veh is eastbound and origin is the other chkpnt
                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                        elseif VEH.drctn==2 && EXWTc(m,2)==0 % veh is westbound and cum.cost to origin of pax=C
                            EXWTc(m,2)=0;
                        elseif VEH.drctn==2 && PaxInfo(m,2)~=C % veh is eastbound and origin is the other chkpnt
                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                        end
                    end
                else
                    EXWTc(m,2)=EXWTc(m,2)-RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,2),6);    % substitute dwellt
                end
                % initiate EXTTc
                EXTTc(m,1:2)=[PaxInfo(m,1),TempCumCost(RtPrsnt(:,1)==PaxInfo(m,3),1)-EXWTc(m,2)]; % difference bewteen cum.cost of destination and origin
                % consider walkt for EXWT
                EXWTc(m,2)=max(0,EXWTc(m,2)-WalkTc(WalkTc(:,1)==PaxInfo(m,4),2));

                if RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,3),5)==3 % if destination is chkpnt
                    % substitute slack time when arriving
                    if drctn==1
                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2); 
                    elseif drctn==2
                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                    end
                else
                    % substitute dwell time
                    EXTTc(m,2)=EXTTc(m,2)-RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,3),6);
                end
            elseif PaxInfo(m,4)==1  % pax is onboard: only update EXTT
                EXTTc(m,1:2)=[PaxInfo(m,1),TempCumCost(RtPrsnt(:,1)==PaxInfo(m,3),1)];
                if RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,3),5)==3
                    if drctn==1
                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                    elseif drctn==2
                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                    end
                else
                    EXTTc(m,2)=EXTTc(m,2)-RtPrsnt(RtPrsnt(:,1)==PaxInfo(m,3),6);
                end
            end
        end

        temppfmcmsr=alpha*sum(EXTTc(:,2))+beta*sum(EXWTc(:,2)); % temporary performance measure (linear combination of expected wait and in-vehicle time)

        % final output
        if sum(TempCost(:,1)) < sum(mincost) && sum(S)==0 && sum(EXWTc(:,2)>maxwait)==0 % alternative requires less cost to cover same passenger within maximum wait time
            if veh1st==1 % current vehicle location is included in RtPrsnt
                mincost = TempCost(2:end,1);        % archive temporary segment travel cost
                mincumcost=TempCumCost(2:end,:);    % archive temporary cumulative segment travel cost
            else % not included
                mincost = TempCost(:,1);
                mincumcost=TempCumCost;                    
            end
            minload = Bload;                    % archive initial vehicle load profile
            minST = BST;                        % archive temporary slack time
            minpfmcmsr = temppfmcmsr;           % archive temporary performance measure
            minEXWT=EXWTc;                      % archive temporary expected wait time
            minEXTT=EXTTc;                      % archive temporary expected in-vehicle time
            minWalkT=WalkTc;                    % archive temporary walking time and other time elements
        end
    end
else
    mincost=inf;    % infeasible of passenger arrival time (later than vehicle departure)
end

% arrange final outputs to candidate route data structure if feasible
if mincost==inf % assign null result because of no appropriate route with new customer
    Aroute=[];
    ARouteCost=[];
    ACumRouteCost=0;
    EXTT=[0,0];
    EXWT=[0,0];
    pfmcmsr=inf;
    AST=ST;
    Aload=inf;
    AWalkT=WalkT;
else % found best route with new customer
    if veh1st==1 % current vehicle location is included in RtPrsnt
        Aroute = RtPrsnt(2:end,:);  % route information
    else
        Aroute = RtPrsnt;
    end
    ARouteCost = mincost;       % segment travel time
    ACumRouteCost=mincumcost;   % cumulative segment travel time
    AST=minST;                  % slack time
    Aload=minload;              % vehicle load profile
    EXWT=minEXWT;               % expected wait time
    EXTT=minEXTT;               % expected in-vehicle time
    AWalkT=minWalkT;            % walking time and other time elements
    pfmcmsr=minpfmcmsr;         % performance measure
end

% arrange final outputs to candidate route data structure
CAND.Rt=Aroute;         % route
CAND.RtCost=[ARouteCost,ACumRouteCost]; % segment travel time and its cumulative time
CAND.perform=pfmcmsr-pfmcmsrb;          % increment of performance measure
CAND.Exwt=EXWT;         % expected wait time
CAND.Extt=EXTT;         % expected in-vehicle time
CAND.Pax=PaxInfo;       % passenger information
CAND.SlackT=AST;        % slack time
CAND.WalkT=AWalkT;      % walking time (including wait and in-vehicle time)

% simulation error check
for i=1:C-1
    if CAND.SlackT(i,2)>CAND.SlackT(i,1) || CAND.perform<0  % slack time violation OR negative performance measure difference
        t        
    end
end
if max(Aload)<Inf
    if max(Aload)>VEH.q     % vehicle load exceeds capacity
        t
    elseif Aload(end,1)>0   % passenger remain at the end of one-way trip
        t
    elseif Aload(1,1)<0     % initial load is negative
        t        
    end
end