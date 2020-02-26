function [CAND]=Insert_Flex(VEH,PAX,alpha,beta,walkspeed,C,t,maxwait)
% % INPUTS % %
% VEH: vehicle data structure
% PAX: passegner data structure
% alpha: weight for in-vehicle time
% beta: weight for wait time
% walkspeed: walking speed
% C: number of checkpoint
% t: current time
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
if (VEH.LocNow(1,1)~=RtPrsnt(1,2) || VEH.LocNow(1,2)~=RtPrsnt(1,3)) && PAX.drctn==VEH.drctn % vehicle did not arrive at the next stop, and new passenger can be added before next stop
    RtPrsnt=[-1,VEH.LocNow,t,0,0,VEH.load;RtPrsnt]; % consider current vehicle loaction as 1st stop of route
    RtCost=[0;RtCost];      % add zero for segment travel cost to current vehicle location
    veh1st=1;	% index of initial vehicle location consideration (0: not included,1:included)
else
    veh1st=0;   % index of initial vehicle location consideration (0: not included,1:included)
end
    
numPaxb=size(Pax,1);	% number of existing pax
if veh1st==1
    % create RtCumCost: cumulative travel cost from current time to departure time of each stop
    if RtPrsnt(2,5)==3 % 1st stop is checkpoint
        RtCumCost(2,1)=RtPrsnt(2,4)-t;  % predetermined departure time - current time
    else % 1st stop is not checkpoing
        RtCumCost(2,1)=RtCost(2,1)+RtPrsnt(2,6); % travel time from current location to 1st stop
    end
    for i=3:size(RtCost,1)
        RtCumCost(i,1)=RtCumCost(i-1,1)+RtCost(i,1)+RtPrsnt(i,6); % accumulate cost (cumulative segment travel time to previous stop + segment travel time to i-th stop + dwell time
        if RtPrsnt(i,5)==3 % i-th stop is checkpoint
            RtCumCost(i,1)=RtPrsnt(i,4)-t;  % predetermined departure time - current time
        end
    end
else
    if RtPrsnt(1,5)==3 % if 1st stop is chkpnt
        RtCumCost(1,1)=RtPrsnt(1,4)-t;  % predetermined departure time - current time
    else % if 1st stop is not chkpnt
        RtCumCost(1,1)=RtCost(1,1)+RtPrsnt(1,6); % travel time from current location to 1st stop
    end
    for i=2:size(RtCost,1)
        RtCumCost(i,1)=RtCumCost(i-1,1)+RtCost(i,1)+RtPrsnt(i,6); % accumulate cost
        if RtPrsnt(i,5)==3 % if i-th stop is chkpnt
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
                    EXWTb(m,2)=EXWTb(m,2)-ST(Pax(m,2)-1,2);
                end
            end
        else % origin is not checkpoint
            EXWTb(m,2)=EXWTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,2),6); % EXWT Step 2-5. CumCost to origin - dwell time
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
    elseif Pax(m,4)==1  % if pax is picked up previously and onboard
        EXTTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,3),1)]; % EXTT Step 1. CumCost to destination
        if RtPrsnt(RtPrsnt(:,1)==Pax(m,3),5)==3 % destination is checkpoint
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

PaxInfo=[Pax;PAX.id,PAX.O(1,1),PAX.D(1,1),0]; % existing and new passenger information [PaxID,OStopID,DStopID,Boarding index]
numPax=size(PaxInfo,1);         % number of passenger
nRtChk=sum(RtPrsnt(:,5)==3);    % remaining number of checkpoint to visit in this direction

% examine required costs between checkpoints
TCost=zeros(size(RtPrsnt,1),2); % actual cost required to reach next points (travel time + dwell time)
if veh1st==1	% current vehicle location is included in RtPrsnt
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
        TCost(1,1) = 1/VEH.v*3600*GridDist(VEH.LocNow,RtPrsnt(1,2:3))+RtPrsnt(1,6); % travel time from current location to 1st stop + dwell time
        VehLoad = VEH.load; % initial vehicle load
    else % vehicle direction is different
        TCost(1,1) = RtPrsnt(1,4)-t; % calculation from route of opposite direction (departure time of 1st chkpnt - current time)
        VehLoad = 0;        % no passenger is onboard
    end
    if drctn==1 % direction is rightward
        TCost(1,2)=C-nRtChk;    % this segment belongs to (C-nRtChk)-th section
        for i=2:size(RtPrsnt,1)
            TCost(i,1) = 1/VEH.v*3600*GridDist(RtPrsnt(i-1,2:3),RtPrsnt(i,2:3))+RtPrsnt(i,6);   % segment travel time + dwell time
            if RtPrsnt(i-1,5)==3    % (i-1)-th stop is checkpoint
                TCost(i,2)=TCost(i-1,2)+1;  % increase checkpoint ID by 1 as vehicle passes one
            else % (i-1)-th stop is not checkpoint
                TCost(i,2)=TCost(i-1,2);    % keep checkpoint ID
            end
        end
    else % direction is rightward
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
ChkCost=zeros(C-1,4); % prepare matrix for section travel time [required travel time,,,]
for i=1:C-1
    if drctn==1     % direction is rightward
        ChkCost(i,1)=sum(TCost(TCost(:,2)==i,1)); % initially required travel time for section before i-th checkpoint
    elseif drctn==2 % direction is leftward
        ChkCost(i,1)=sum(TCost(TCost(:,2)==C-i,1));
    end
end

% build route including new pax regarding existence of route
if PAX.type==1 && sum(RtPrsnt(:,1)==PAX.O(1,1)) && sum(RtPrsnt(:,1)==PAX.D(1,1))    % new customer's OD stops are checkpoints
    owalkt=1/walkspeed*3600*GridDist(PAX.O(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.O(1,1),2:3));   % walking time from true origin to O checkpoint
    dwalkt=1/walkspeed*3600*GridDist(PAX.D(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.D(1,1),2:3));   % walking time from true origin to O checkpoint
    if t+owalkt<RtPrsnt(RtPrsnt(:,1)==PAX.O(1,1),4) % passenger can reach checkpoint before vehicle arrival
        % other parameters remain the same because route doesn't change except for load
        re=size(RtPrsnt,1); % length of current route
        RtInfo=[(1:re)',RtPrsnt];    % route information (Sequence, Route)

        % find sequences of O stop and D stop
        p=RtInfo(RtInfo(:,2)==PAX.O(1,1),1);    % sequence of O checkpoint
        RtInfo(p,8)=RtInfo(p,8)+PAX.O(1,7);     % load pax
        q=RtInfo(RtInfo(:,2)==PAX.D(1,1),1);    % sequence of D stop
        RtInfo(q,8)=RtInfo(q,8)+PAX.D(1,7);     % unload pax        

        % update vehicle load profile
        Load(1,1)=RtInfo(1,8); % load profile
        for i=2:re
            Load(i,1)=Load(i-1,1)+RtInfo(i,8);  % accumulate load per stop
        end
        CumCost=RtCumCost;  % CumCost remains the same
        WTs=[PAX.id,owalkt,dwalkt,0,0]; % array for WalkT
    else
        Load=999; % assign excessive load to vehicle to reject 
    end

    % reflect addition of new passenger if vehicle capacity is not exceeded
    if max(Load)<=VEH.q
        if veh1st==1
            Aroute=RtInfo(2:end,2:8);       % route information except for current vehicle location
            ARouteCost=RtCost(2:end,:);     % segment travel time except for current vehicle location
            ACumRouteCost=CumCost(2:end,:); % cumulative segment travel time except for current vehicle location
        else
            Aroute=RtInfo(:,2:8);
            ARouteCost=RtCost;
            ACumRouteCost=CumCost;
        end
        EXWT=zeros(numPax,2);   % space for expected wait time of passengers
        EXTT=zeros(numPax,2);   % space for expected in-vehicle time of passengers
        WalkT=[WalkT;WTs];      % walking time of passengers (wait and in-vehicle time included)
        for m=2:numPax
            if PaxInfo(m,4)==0  % passenger is not picked up yet
                EXWT(m,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,2),1)];  % EXWT Step 1. CumCost to origin
                if Aroute(Aroute(:,1)==PaxInfo(m,2),5)==3    % origin is checkpoint
                    if drctn==1 % direction is rightward
                        if VEH.drctn==2 && PaxInfo(m,2)==1      % vehicle is opposite direction and origin is a checkpoint #1
                            EXWT(m,2)=EXWT(m,2)-VEH.SlackT2(C-1,2); % EXWT Step 2-1. CumCost to origin - last slack time in opposite direction
                        elseif VEH.drctn==2 && PaxInfo(m,2)~=1  % origin is a checkpoint other than #1
                            EXWT(m,2)=EXWT(m,2)-ST(PaxInfo(m,2)-1,2);   % EXWT Step 2-2. CumCost to origin - slack time in same direction
                        elseif VEH.drctn==1 && EXWT(m,2)==0 % vehicle is rightward and expected wait time is zero (passenger is onboard but waiting for departure)
                            EXWT(m,2)=0;    % EXWT Step 2-3. expected wait time remains zero
                        elseif VEH.drctn==1 && PaxInfo(m,2)~=1  % vehicle is rightward and origin is checkpoint other than #1
                            EXWT(m,2)=EXWT(m,2)-ST(PaxInfo(m,2)-1,2);   % EXWT Step 2-4. CumCost to origin - slack time of that checkpoint
                        end
                    elseif drctn==2 % direction is leftward
                        if VEH.drctn==1 && PaxInfo(m,2)==C
                            EXWT(m,2)=EXWT(m,2)-VEH.SlackT1(C-1,2);
                        elseif VEH.drctn==1 && PaxInfo(m,2)~=C
                            EXWT(m,2)=EXWT(m,2)-ST(C-PaxInfo(m,2),2);
                        elseif VEH.drctn==2 && EXWT(m,2)==0
                            EXWT(m,2)=0;
                        elseif VEH.drctn==2 && PaxInfo(m,2)~=C
                            EXWT(m,2)=EXWT(m,2)-ST(C-PaxInfo(m,2),2);
                        end
                    end
                else % origin is not checkpoint
                    EXWT(m,2)=EXWT(m,2)-Aroute(Aroute(:,1)==PaxInfo(m,2),6);    % EXWT Step 2-5. CumCost to origin - dwell time
                end
                EXTT(m,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,3),1)-EXWT(m,2)];    % EXTT Step 1. CumCost to destination - EXWT
                EXWT(m,2)=max(0,EXWT(m,2)-WalkT(WalkT(:,1)==PaxInfo(m,1),2));   % EXWT Step 3. deduct walking time from expected wait time

                if Aroute(Aroute(:,1)==PaxInfo(m,3),5)==3   % destination is checkpoint
                    if drctn==1 % direction is rightward
                        EXTT(m,2)=EXTT(m,2)-ST(PaxInfo(m,3)-1,2);   % EXTT Step 2-1. EXTT - slack time
                    elseif drctn==2 % direction is leftward
                        EXTT(m,2)=EXTT(m,2)-ST(C-PaxInfo(m,3),2);   % EXTT Step 2-1. EXTT - slack time
                    end
                else % destination is not checkpoint
                    EXTT(m,2)=EXTT(m,2)-Aroute(Aroute(:,1)==PaxInfo(m,3),6);    % EXTT Step 2-2. EXTT - dwell time
                end

            elseif PaxInfo(m,4)==1  % if pax is picked up previously and onboard
                EXTT(m,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,3),1)];  % EXTT Step 1. CumCost to destination
                if Aroute(Aroute(:,1)==PaxInfo(m,3),5)==3   % destination is checkpoint
                    if drctn==1 % direction is rightward
                        EXTT(m,2)=EXTT(m,2)-ST(PaxInfo(m,3)-1,2);   % EXTT Step 2-1. EXTT - slack time
                    elseif drctn==2 % direction is leftward
                        EXTT(m,2)=EXTT(m,2)-ST(C-PaxInfo(m,3),2);   % EXTT Step 2-1. EXTT - slack time
                    end
                else % destination is not checkpoint
                    EXTT(m,2)=EXTT(m,2)-Aroute(Aroute(:,1)==PaxInfo(m,3),6);    % EXTT Step 2-2. EXTT - dwell time
                end
            end
        end
        pfmcmsr=alpha*sum(EXTT(:,2))+beta*sum(EXWT(:,2));   % performance measure calculation (linear combination of wait and in-vehicle time)
        Aload=Load;     % final vehicle load profile
        AWalkT=WalkT;   % final walking time and other time elements
    else % assign null result if capacity condition violated
        Aroute=[];
        ARouteCost=[];
        ACumRouteCost=0;
        EXTT=[0,0];
        EXWT=[0,0];
        pfmcmsr=inf;
        Aload=inf;
        AWalkT=WalkT;
    end
    AST=ST; % return same ST because it does not change if passenger type is 1
else % PAX.type==2,3,4
    Route=[];   % route information (StopID,X,Y,stop type(1/2/3),PaxID)(Stop type 1: non-regular pickup, 2: non-regular drop-off, 3: checkpoint)
    if PAX.type==2 && sum(RtPrsnt(:,1)==PAX.O(1,1))     % passenger type is 2 (PND), and O checkpoint is included in route
        Route=[RtPrsnt;PAX.D];          % add non-regular drop-off
    elseif PAX.type==3 && sum(RtPrsnt(:,1)==PAX.D(1,1)) % passenger type is 3 (NPD), and D checkpoint is included in route
        Route=[RtPrsnt;PAX.O];          % add non-regular pickup
    elseif PAX.type==4                                  % passenger type is 4 (NPND)
        Route=[RtPrsnt;PAX.O;PAX.D];    % add non-regular pickup and drop-off
    end

    % build route for passenger type 2,3,4
    if size(Route,1)>0
        rt=size(Route,1);       % length of route including new O and/or D to existing route
        RtInfo=[(1:rt)',Route];	% route information (ID, Route(7))

        % create distance matrix
        DistMat=zeros(rt,rt);   % distance matrix between stops
        for i=1:rt
            for j=1:rt
                if i<j
                    DistMat(i,j)=1/VEH.v*3600*GridDist(RtInfo(i,3:4),RtInfo(j,3:4));    % calculate distance
                elseif i>j
                    DistMat(i,j)=DistMat(j,i);  % matrix is symmetric
                end
            end
        end
        re=size(RtPrsnt,1); % length of current route
        Broute=(1:re)';     % label of current route
        mincost = inf;
        minroute = Broute;
        if PAX.type==2 % P-ND, adding non-regular drop-off only
            owalkt=1/walkspeed*3600*GridDist(PAX.O(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.O(1,1),2:3));   % walking time from true origin to O checkpoint
            dwalkt=0;   % no walking for destination
            if t+owalkt<RtPrsnt(RtPrsnt(:,1)==PAX.O(1,1),4) % passenger arrival time < vehicle departure time
                % find sequences of O stop and D stop
                p=RtInfo(RtInfo(:,2)==PAX.O(1,1),1);    % sequence of O checkpoint
                RtInfo(p,8)=RtInfo(p,8)+PAX.O(1,7);     % load pax
                % vehicle load profile
                Bload(1,1)=RtInfo(1,8); % initiate current vehicle load profile
                if veh1st~=1
                    Bload(1,1)=Bload(1,1)+VehLoad;
                end
                for i=2:re
                    Bload(i,1)=Bload(i-1,1)+RtInfo(i,8);    % accumulate route
                end

                % insertion heuristic only for non-regular drop-off
                for j=p:re-1
                    BST=ST; % import slack time
                    TempLoad = [Bload(1:j,1); Bload(j,1)+PAX.D(1,7); Bload(j+1:re,1)+PAX.D(1,7)];   % temporary load with destination of new passenger at j-th sequence
                    TempRoute = [Broute(1:j,1); re+1; Broute(j+1:re,1)];                            % temporary route with destination of new passenger at j-th sequence
                    if max(TempLoad) <= VEH.q % vehicle capacity restriction
                        % initialize temporary segment travel time and cumulative segment travel time
                        rs2 = size(TempRoute,1);    % length of temporary route
                        TempCost=zeros(rs2,1);      % prepare matrix for temporary segment travel time
                        TempCumCost=zeros(rs2,1);   % prepare matrix for temporary cumulative segment travel time
                        if veh1st~=1 % current vehicle location is not included in RtPrsnt
                            TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(TempRoute(1,1),3:4))+RtInfo(TempRoute(1,1),7);	% segment travel time + dwell time
                            if RtInfo(TempRoute(1,1),6)==3	% 1st stop is checkpoint
                                TempCumCost(1,1)=RtInfo(TempRoute(1,1),5)-t;    % departure time - current time
                            else % not checkpoint
                                TempCumCost(1,1)=TempCost(1,1); % same as travel time of 1st segment
                            end
                        end
                        S=zeros(1,C);   % feasibility indicator array
                        if drctn==1 % rightward
                            TempCost(1,2)=C-nRtChk+1;   % checkpoint ID that involves 1st stop in its section
                            TempCost(1,3)=min(0,RtInfo(TempRoute(1,1),3)-VEH.LocNow(1,1)); % backtracking distance (difference between current location and first stop)
                        else % leftward
                            TempCost(1,2)=nRtChk;       % checkpoint ID that involves 1st stop in its section
                            TempCost(1,3)=min(0,VEH.LocNow(1,1)-RtInfo(TempRoute(1,1),3)); % backtracking distance (difference between current location and first stop)
                        end

                        % build temporary segment travel time and cumulative segment travel time
                        for m = 1:rs2-1
                            TempCost(m+1,1)=DistMat(TempRoute(m,1),TempRoute(m+1,1))+RtInfo(TempRoute(m+1,1),7);    % travel time + dwell time
                            if drctn==1 % rightward
                                TempCost(m+1,3)=min(0,RtInfo(TempRoute(m+1,1),3)-RtInfo(TempRoute(m,1),3)); % backtracking distance
                            else % leftward
                                TempCost(m+1,3)=min(0,RtInfo(TempRoute(m,1),3)-RtInfo(TempRoute(m+1,1),3)); % backtracking distance
                            end
                            if RtInfo(TempRoute(m,1),6)==3  % m-th stop is checkpoint
                                if drctn==1 % rightward
                                    TempCost(m+1,2)=TempCost(m,2)+1;    % increase checkpoint ID as vehicles passes it
                                else % leftward
                                    TempCost(m+1,2)=TempCost(m,2)-1;    % decrease checkpoint ID as vehicles passes it
                                end
                            else
                                TempCost(m+1,2)=TempCost(m,2);  % keep checkpoint ID
                            end
                            TempCumCost(m+1,1)=TempCumCost(m,1)+TempCost(m+1,1)+RtInfo(TempRoute(m+1,1),7); % temporary cumulative segment travel time to previous stop + segment travel time + dwell time
                            if RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)<=RtInfo(TempRoute(m+1,1),5)-t    % vehicle can arrive at checkpoint and finish dwelling before departure time
                                TempCumCost(m+1,1)=RtInfo(TempRoute(m+1,1),5)-t;    % replace temporary cumulative segment travel time with departure time - current time
                            elseif RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)>RtInfo(TempRoute(m+1,1),5)-t % vehicle cannot
                                S=ones(1,C);    % infeasible
                                break
                            end
                        end

                        % feasibility check: slack time of sections
                        if sum(S)==0    % vehicle can comply timetable
                            CP=RtInfo(RtInfo(:,6)==3,[2,1]);    %[checkpoint ID, route ID)
                            for m=1:nRtChk
                                CP(m,3)=find(TempRoute(:,1)==CP(m,2));  % find sequence of checkpoint
                                if m==1 % m-th checkpoint is 1st checkpoint vehicle visits
                                    CP(m,4)=TempCumCost(CP(m,3),1); % pure cumulative segment travel time
                                else
                                    CP(m,4)=TempCumCost(CP(m,3),1)-TempCumCost(CP(m-1,3),1);    % cumulative segment travel time from which that of previous checkpoint is deducted
                                end
                            end
                            if size(CP,1)==C % all checkpoints are on route
                                for m=1:C-1
                                    ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1,1),1));     % required section travel time after insertion
                                    ChkCost(m,3)=CP(m+1,4)-RtInfo(CP(m+1,2),7);                 % available slack time
                                    ChkCost(m,4)=sum(TempCost(TempCost(:,2)==CP(m+1,1),3));     % backtracking distance
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
                                    ChkCost(m,3)=CP(m+1-C+nRtChk,4)-RtInfo(CP(m+1-C+nRtChk,2),7);
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

                        % feasibility check: wait time limit
                        Croute=zeros(size(TempRoute,1),7);  % prepare matrix for checking route
                        for m=1:size(TempRoute)
                            Croute(m,:)=RtInfo(TempRoute(m,1),2:8);   % update Croute
                        end
                        CCumRouteCost=TempCumCost;  % import cumulative segment travel time
                        EXWTc=zeros(numPax,2);  % space for expected wait time
                        EXTTc=zeros(numPax,2);  % space for expected in-vehicle time
                        WalkTc=[WalkT;PAX.id,owalkt,dwalkt,0,0];    % walking time and other time elements
                        for m=2:numPax
                            if PaxInfo(m,4)==0
                                EXWTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,2),1)];
                                if Croute(Croute(:,1)==PaxInfo(m,2),5)==3
                                    if drctn==1
                                        if VEH.drctn==2 && PaxInfo(m,2)==1
                                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT2(C-1,2);
                                        elseif VEH.drctn==2 && PaxInfo(m,2)~=1
                                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                        elseif VEH.drctn==1 && EXWTc(m,2)==0
                                            EXWTc(m,2)=0;
                                        elseif VEH.drctn==1 && PaxInfo(m,2)~=1
                                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                        end
                                    elseif drctn==2
                                        if VEH.drctn==1 && PaxInfo(m,2)==C
                                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT1(C-1,2);
                                        elseif VEH.drctn==1 && PaxInfo(m,2)~=C
                                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                        elseif VEH.drctn==2 && EXWTc(m,2)==0
                                            EXWTc(m,2)=0;
                                        elseif VEH.drctn==2 && PaxInfo(m,2)~=C
                                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                        end
                                    end
                                else
                                    EXWTc(m,2)=EXWTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,2),6);
                                end
                                EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)-EXWTc(m,2)];
                                EXWTc(m,2)=max(0,EXWTc(m,2)-WalkTc(WalkTc(:,1)==PaxInfo(m,4),2));
                                if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                    if drctn==1
                                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                    elseif drctn==2
                                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                    end
                                else
                                    EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                                end
                            elseif PaxInfo(m,4)==1
                                EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)];
                                if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                    if drctn==1
                                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                    elseif drctn==2
                                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                    end
                                else
                                    EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                                end
                            end
                        end

                        temppfmcmsr=alpha*sum(EXTTc(:,2))+beta*sum(EXWTc(:,2)); % temporary performance measure (linear combination of expected wait and in-vehicle time)

                        if sum(TempCost(:,1)) < sum(mincost) && sum(S)==0 && sum(EXWTc(:,2)>maxwait)==0 % alternative requires less cost to cover same passenger within maximum wait time
                            if veh1st==1    % current vehicle location is included in RtPrsnt
                                mincost = TempCost(2:end,1);        % archive temporary segment travel cost
                                minroute = TempRoute(2:end,:);      % archive temporary route
                                mincumcost=TempCumCost(2:end,:);    % archive temporary cumulative segment travel cost
                            else % not included
                                mincost = TempCost(:,1);
                                minroute = TempRoute;
                                mincumcost=TempCumCost;
                            end
                            minload = TempLoad;                 % archive temporary vehicle load profile                                
                            minST = BST;                        % archive temporary slack time
                            minpfmcmsr = temppfmcmsr;           % archive temporary performance measure
                            minEXWT=EXWTc;                      % archive temporary expected wait time
                            minEXTT=EXTTc;                      % archive temporary expected in-vehicle time
                            minWalkT=WalkTc;                    % archive temporary walking time and other time elements
                        end
                    end
                end
            else
                mincost=inf;    % infeasible of passenger arrival time (later than vehicle departure)
            end

        elseif PAX.type==3 % NP-D, adding non-regular pickup only
            owalkt=0;   % no walking for origin
            dwalkt=1/walkspeed*3600*GridDist(PAX.D(1,2:3),RtPrsnt(RtPrsnt(:,1)==PAX.D(1,1),2:3));   % walking time from true destination to D checkpoint

            % find sequences of O stop and D stop
            q=RtInfo(RtInfo(:,2)==PAX.D(1,1),1);    % sequence of D checkpoint
            RtInfo(q,8)=RtInfo(q,8)+PAX.D(1,7);     % unload pax
            % vehicle load profile
            Bload(1,1)=RtInfo(1,8);	% initiate current vehicle load profile
            if veh1st~=1 % current vehicle location is not included in RtPrsnt
                Bload(1,1)=Bload(1,1)+VehLoad;  % reflect current load
            end
            for i=2:re
                Bload(i,1)=Bload(i-1,1)+RtInfo(i,8);    % accumulate route
            end

            % insertion heuristic only for non-regular pickup
            for j = 1:q-1
                BST=ST; % import slack time
                TempLoad = [Bload(1:j,1); Bload(j,1)+PAX.O(1,7); Bload(j+1:re,1)+PAX.O(1,7)];   % temporary load with origin of new passenger at j-th sequence
                TempRoute = [Broute(1:j,1); re+1; Broute(j+1:re,1)];                            % temporary route with origin of new passenger at j-th sequence
                if max(TempLoad) <= VEH.q %vehicle capacity restriction
                    % initialize temporary segment travel time and cumulative segment travel time
                    rs2 = size(TempRoute,1);    % length of temporary route
                    TempCost=zeros(rs2,1);      % prepare matrix for temporary segment travel time
                    TempCumCost=zeros(rs2,1);   % prepare matrix for temporary cumulative segment travel time
                    if veh1st~=1 % current vehicle location is not included in RtPrsnt
                        TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(TempRoute(1,1),3:4))+RtInfo(TempRoute(1,1),7); % current vehicle location is not included in RtPrsnt
                        if RtInfo(TempRoute(1,1),6)==3   % 1st stop is checkpoint
                            TempCumCost(1,1)=RtInfo(TempRoute(1,1),5)-t; % departure time - current time
                        else % not checkpoint
                            TempCumCost(1,1)=TempCost(1,1); % same as travel time of 1st segment
                        end
                    end
                    S=zeros(1,C);   % feasibility indicator array

                    if drctn==1 % rightward
                        TempCost(1,2)=C-nRtChk+1;   % checkpoint ID that involves 1st stop in its section
                        TempCost(1,3)=min(0,RtInfo(TempRoute(1,1),3)-VEH.LocNow(1,1)); % backtracking distance
                    else % leftward
                        TempCost(1,2)=nRtChk;       % checkpoint ID that involves 1st stop in its section
                        TempCost(1,3)=min(0,VEH.LocNow(1,1)-RtInfo(TempRoute(1,1),3)); % backtracking distance
                    end

                    % build temporary segment travel time and cumulative segment travel time
                    for m = 1:rs2-1
                        TempCost(m+1,1)=DistMat(TempRoute(m,1),TempRoute(m+1,1))+RtInfo(TempRoute(m+1,1),7);    % travel time + dwell time
                        if drctn==1 % rightward
                            TempCost(m+1,3)=min(0,RtInfo(TempRoute(m+1,1),3)-RtInfo(TempRoute(m,1),3)); % backtracking distance
                        else % leftward
                            TempCost(m+1,3)=min(0,RtInfo(TempRoute(m,1),3)-RtInfo(TempRoute(m+1,1),3)); % backtracking distance
                        end
                        if RtInfo(TempRoute(m,1),6)==3  % m-th stop is checkpoint
                            if drctn==1 % rightward
                                TempCost(m+1,2)=TempCost(m,2)+1;    % increase checkpoint ID as vehicles passes it
                            else % leftward
                                TempCost(m+1,2)=TempCost(m,2)-1;    % decrease checkpoint ID as vehicles passes it
                            end
                        else
                            TempCost(m+1,2)=TempCost(m,2);  % keep checkpoint ID
                        end
                        TempCumCost(m+1,1)=TempCumCost(m,1)+TempCost(m+1,1)+RtInfo(TempRoute(m+1,1),7); % temporary cumulative segment travel time to previous stop + segment travel time + dwell time
                        if RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)<=RtInfo(TempRoute(m+1,1),5)-t    % vehicle can arrive at checkpoint and finish dwelling before departure time
                            TempCumCost(m+1,1)=RtInfo(TempRoute(m+1,1),5)-t;    % replace temporary cumulative segment travel time with departure time - current time
                        elseif RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)>RtInfo(TempRoute(m+1,1),5)-t % vehicle cannot
                            S=ones(1,C);    % infeasible
                            break
                        end
                    end

                    % feasibility check: slack time of sections
                    if sum(S)==0    % vehicle can comply timetable
                        CP=RtInfo(RtInfo(:,6)==3,[2,1]); %[checkpoint ID, route ID)
                        for m=1:nRtChk
                            CP(m,3)=find(TempRoute(:,1)==CP(m,2));  % find sequence of checkpoint
                            if m==1 % m-th checkpoint is 1st checkpoint vehicle visits
                                CP(m,4)=TempCumCost(CP(m,3),1); % pure cumulative segment travel time
                            else
                                CP(m,4)=TempCumCost(CP(m,3),1)-TempCumCost(CP(m-1,3),1);    % cumulative segment travel time from which that of previous checkpoint is deducted
                            end
                        end
                        if size(CP,1)==C % all checkpoints are on route
                            for m=1:C-1
                                ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1,1),1));     % required time after insertion
                                ChkCost(m,3)=CP(m+1,4)-RtInfo(CP(m+1,2),7);                 % available slack time
                                ChkCost(m,4)=sum(TempCost(TempCost(:,2)==CP(m+1,1),3));     % backtracking distance
                                if ChkCost(m,2)>ChkCost(m,3) || ChkCost(m,2)-ChkCost(m,1)>BST(m,3) || BST(m,5)+ChkCost(m,4)<0
                                    % required time is longer than available slack time OR increasing time is longer than usable slack time OR backtracking distance is longer than remaining amount
                                    S=ones(1,C);    % infeasible
                                    break
                                else % satisfy slack time condition
                                    BST(m,2)=BST(m,2)-(ChkCost(m,2)-ChkCost(m,1));  % update available slack time
                                    BST(m,5)=BST(m,5)+ChkCost(m,4);                 % update available backtracking
                                    if BST(m,2)<BST(m,3) % available slack time is shorter than usable slack time
                                        BST(m,3)=BST(m,2);  % cap usable slack time with available slack time
                                    end
                                end
                            end
                        else % some checkpoints are on route
                            for m=C-nRtChk:C-1
                                ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1-C+nRtChk,1),1));
                                ChkCost(m,3)=CP(m+1-C+nRtChk,4)-RtInfo(CP(m+1-C+nRtChk,2),7);
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

                    % feasibility check: wait time limit
                    Croute=zeros(size(TempRoute,1),7);   % prepare matrix for checking route
                    for m=1:size(TempRoute)
                        Croute(m,:)=RtInfo(TempRoute(m,1),2:8);   % update Croute
                    end
                    CCumRouteCost=TempCumCost;	% import cumulative segment travel time
                    EXWTc=zeros(numPax,2);  % space for expected wait time
                    EXTTc=zeros(numPax,2);  % space for expected in-vehicle time
                    WalkTc=[WalkT;PAX.id,owalkt,dwalkt,0,0];    % walking time and other time elements
                    for m=2:numPax
                        if PaxInfo(m,4)==0
                            EXWTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,2),1)];
                            if Croute(Croute(:,1)==PaxInfo(m,2),5)==3
                                if drctn==1
                                    if VEH.drctn==2 && PaxInfo(m,2)==1
                                        EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT2(C-1,2);
                                    elseif VEH.drctn==2 && PaxInfo(m,2)~=1
                                        EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                    elseif VEH.drctn==1 && EXWTc(m,2)==0
                                        EXWTc(m,2)=0;
                                    elseif VEH.drctn==1 && PaxInfo(m,2)~=1
                                        EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                    end
                                elseif drctn==2
                                    if VEH.drctn==1 && PaxInfo(m,2)==C
                                        EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT1(C-1,2);
                                    elseif VEH.drctn==1 && PaxInfo(m,2)~=C
                                        EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                    elseif VEH.drctn==2 && EXWTc(m,2)==0
                                        EXWTc(m,2)=0;
                                    elseif VEH.drctn==2 && PaxInfo(m,2)~=C
                                        EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                    end
                                end
                            else
                                EXWTc(m,2)=EXWTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,2),6);
                            end
                            EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)-EXWTc(m,2)];
                            EXWTc(m,2)=max(0,EXWTc(m,2)-WalkTc(WalkTc(:,1)==PaxInfo(m,4),2));

                            if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                if drctn==1
                                    EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                elseif drctn==2
                                    EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                end
                            else
                                EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                            end

                        elseif PaxInfo(m,4)==1
                            EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)];
                            if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                if drctn==1
                                    EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                elseif drctn==2
                                    EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                end
                            else
                                EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                            end
                        end
                    end

                    temppfmcmsr=alpha*sum(EXTTc(:,2))+beta*sum(EXWTc(:,2)); % temporary performance measure (linear combination of expected wait and in-vehicle time)

                    if sum(TempCost(:,1)) < sum(mincost) && sum(S)==0 && sum(EXWTc(:,2)>maxwait)==0
                        if veh1st==1    % current vehicle location is included in RtPrsnt
                            mincost = TempCost(2:end,1);        % archive temporary segment travel cost
                            minroute = TempRoute(2:end,:);      % archive temporary route
                            mincumcost=TempCumCost(2:end,:);    % archive temporary cumulative segment travel cost
                        else % not included
                            mincost = TempCost(:,1);
                            minroute = TempRoute;
                            mincumcost=TempCumCost;
                        end
                        minload = TempLoad;                 % archive temporary vehicle load profile
                        minST = BST;                        % archive temporary slack time
                        minpfmcmsr = temppfmcmsr;           % archive temporary performance measure
                        minEXWT=EXWTc;                      % archive temporary expected wait time
                        minEXTT=EXTTc;                      % archive temporary expected in-vehicle time
                        minWalkT=WalkTc;                    % archive temporary walking time and other time elements
                    end
                end
            end
        elseif PAX.type==4 % NP-ND, adding non-regular pickup and drop-off point
            owalkt=0;   % no walking for origin
            dwalkt=0;   % no walking for destination

            % vehicle load profile
            Bload(1,1)=RtInfo(1,8); % initiate current vehicle load profile
            if veh1st~=1 % current vehicle location is included in RtPrsnt
                Bload(1,1)=Bload(1,1)+VehLoad;  % reflect current load
            end
            for i=2:re
                Bload(i,1)=Bload(i-1,1)+RtInfo(i,8);    % accumulate route
            end

            % insertion heuristic for non-regular pickup and drop-off point
            for j = 1:re-1      % index for temporary sequence of non-regular pickup
                for k = j+1:re  % index for temporary sequence of non-regular drop-off
                    BST=ST;
                    TempLoad = [Bload(1:j,1); Bload(j,1)+PAX.O(1,7); Bload(j+1:re,1)+PAX.O(1,7)];               % temporary load with destination of non-regular pickup at j-th sequence
                    TempRoute = [Broute(1:j,1); re+1; Broute(j+1:re,1)];                    % temporary route with destination of non-regular pickup at j-th sequence
                    TempLoad = [TempLoad(1:k,1); TempLoad(k,1)+PAX.D(1,7); TempLoad(k+1:re+1,1)+PAX.D(1,7)];    % temporary load with destination of non-regular drop-off at k-th sequence
                    TempRoute = [TempRoute(1:k,1); re+2; TempRoute(k+1:re+1,1)];            % temporary route with destination of non-regular pickup at k-th sequence
                    if max(TempLoad) <= VEH.q % vehicle capacity restriction
                        rs2 = size(TempRoute,1);	% length of temporary route
                        TempCost=zeros(rs2,1);      % prepare matrix for temporary segment travel time
%                         TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(TempRoute(1,1),3:4))+RtInfo(TempRoute(1,1),7);
                        TempCumCost=zeros(rs2,1);   % prepare matrix for temporary cumulative segment travel time
                        if veh1st~=1
                            TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(TempRoute(1,1),3:4))+RtInfo(TempRoute(1,1),7);
                            if RtInfo(TempRoute(1,1),6)==3
                                TempCumCost(1,1)=RtInfo(TempRoute(1,1),5)-t;
                            else
                                TempCumCost(1,1)=TempCost(1,1);
                            end
                        end
                        S=zeros(1,C);   % feasibility indicator array
                        if drctn==1 % rightward
                            TempCost(1,2)=C-nRtChk+1;   % checkpoint ID that involves 1st stop in its section
                            TempCost(1,3)=min(0,RtInfo(TempRoute(1,1),3)-VEH.LocNow(1,1)); % backtracking distance
                        else % leftward
                            TempCost(1,2)=nRtChk;       % checkpoint ID that involves 1st stop in its section
                            TempCost(1,3)=min(0,VEH.LocNow(1,1)-RtInfo(TempRoute(1,1),3)); % backtracking distance
                        end

                        % build temporary segment travel time and cumulative segment travel time
                        for m = 1:rs2-1
                            TempCost(m+1,1)=DistMat(TempRoute(m,1),TempRoute(m+1,1))+RtInfo(TempRoute(m+1,1),7);    % travel time + dwell time
                            if drctn==1 % rightward
                                TempCost(m+1,3)=min(0,RtInfo(TempRoute(m+1,1),3)-RtInfo(TempRoute(m,1),3)); % backtracking distance
                            else % leftward
                                TempCost(m+1,3)=min(0,RtInfo(TempRoute(m,1),3)-RtInfo(TempRoute(m+1,1),3)); % backtracking distance
                            end
                            if RtInfo(TempRoute(m,1),6)==3  % m-th stop is checkpoint
                                if drctn==1 % rightward
                                    TempCost(m+1,2)=TempCost(m,2)+1;    % increase checkpoint ID as vehicles passes it
                                else % leftward
                                    TempCost(m+1,2)=TempCost(m,2)-1;    % decrease checkpoint ID as vehicles passes it
                                end
                            else
                                TempCost(m+1,2)=TempCost(m,2);  % keep checkpoint ID
                            end
                            TempCumCost(m+1,1)=TempCumCost(m,1)+TempCost(m+1,1); % temporary cumulative segment travel time to previous stop + segment travel time + dwell time
                            if RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)<=RtInfo(TempRoute(m+1,1),5)-t    % vehicle can arrive at checkpoint and finish dwelling before departure time
                                TempCumCost(m+1,1)=RtInfo(TempRoute(m+1,1),5)-t;    % replace temporary cumulative segment travel time with departure time - current time
                            elseif RtInfo(TempRoute(m+1,1),6)==3 && TempCumCost(m+1,1)>RtInfo(TempRoute(m+1,1),5)-t % vehicle cannot
                                S=ones(1,C);    % infeasible
                                break
                            end
                        end

                        % feasibility check: slack time of sections
                        if sum(S)==0    % vehicle can comply timetable
                            CP=RtInfo(RtInfo(:,6)==3,[2,1]); %[checkpoint ID, route ID)
                            for m=1:nRtChk
                                CP(m,3)=find(TempRoute(:,1)==CP(m,2));  % find sequence of checkpoint
                                if m==1 % m-th checkpoint is 1st checkpoint vehicle visits
                                    CP(m,4)=TempCumCost(CP(m,3),1); % pure cumulative segment travel time
                                else
                                    CP(m,4)=TempCumCost(CP(m,3),1)-TempCumCost(CP(m-1,3),1);    % cumulative segment travel time from which that of previous checkpoint is deducted
                                end
                            end

                            if size(CP,1)==C % all checkpoints are on route
                                for m=1:C-1
                                    ChkCost(m,2)=sum(TempCost(TempCost(:,2)==CP(m+1,1),1));     % required time after insertion
                                    ChkCost(m,3)=CP(m+1,4)-RtInfo(CP(m+1,2),7);                 % available slack time
                                    ChkCost(m,4)=sum(TempCost(TempCost(:,2)==CP(m+1,1),3));     % backtracking distance
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
                                    ChkCost(m,3)=CP(m+1-C+nRtChk,4)-RtInfo(CP(m+1-C+nRtChk,2),7);
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

                        % feasibility check: wait time limit
                        Croute=zeros(size(TempRoute,1),7);  % prepare matrix for checking route
                        for m=1:size(TempRoute)
                            Croute(m,:)=RtInfo(TempRoute(m,1),2:8);   % update Croute
                        end
                        CCumRouteCost=TempCumCost;  % import cumulative segment travel time
                        EXWTc=zeros(numPax,2);  % space for expected wait time
                        EXTTc=zeros(numPax,2);  % space for expected in-vehicle time
                        WalkTc=[WalkT;PAX.id,owalkt,dwalkt,0,0];    % walking time and other time elements
                        for m=2:numPax
                            if PaxInfo(m,4)==0
                                EXWTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,2),1)];
                                if Croute(Croute(:,1)==PaxInfo(m,2),5)==3
                                    if drctn==1
                                        if VEH.drctn==2 && PaxInfo(m,2)==1
                                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT2(C-1,2);
                                        elseif VEH.drctn==2 && PaxInfo(m,2)~=1
                                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                        elseif VEH.drctn==1 && EXWTc(m,2)==0
                                            EXWTc(m,2)=0;
                                        elseif VEH.drctn==1 && PaxInfo(m,2)~=1
                                            EXWTc(m,2)=EXWTc(m,2)-ST(PaxInfo(m,2)-1,2);
                                        end
                                    elseif drctn==2
                                        if VEH.drctn==1 && PaxInfo(m,2)==C
                                            EXWTc(m,2)=EXWTc(m,2)-VEH.SlackT1(C-1,2);
                                        elseif VEH.drctn==1 && PaxInfo(m,2)~=C
                                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                        elseif VEH.drctn==2 && EXWTc(m,2)==0
                                            EXWTc(m,2)=0;
                                        elseif VEH.drctn==2 && PaxInfo(m,2)~=C
                                            EXWTc(m,2)=EXWTc(m,2)-ST(C-PaxInfo(m,2),2);
                                        end
                                    end
                                else
                                    EXWTc(m,2)=EXWTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,2),6);
                                end
                                EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)-EXWTc(m,2)];
                                EXWTc(m,2)=max(0,EXWTc(m,2)-WalkTc(WalkTc(:,1)==PaxInfo(m,4),2));

                                if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                    if drctn==1
                                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                    elseif drctn==2
                                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                    end
                                else
                                    EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                                end

                            elseif PaxInfo(m,4)==1
                                EXTTc(m,1:2)=[PaxInfo(m,1),CCumRouteCost(Croute(:,1)==PaxInfo(m,3),1)];
                                if Croute(Croute(:,1)==PaxInfo(m,3),5)==3
                                    if drctn==1
                                        EXTTc(m,2)=EXTTc(m,2)-ST(PaxInfo(m,3)-1,2);
                                    elseif drctn==2
                                        EXTTc(m,2)=EXTTc(m,2)-ST(C-PaxInfo(m,3),2);
                                    end
                                else
                                    EXTTc(m,2)=EXTTc(m,2)-Croute(Croute(:,1)==PaxInfo(m,3),6);
                                end
                            end
                        end

                        temppfmcmsr=alpha*sum(EXTTc(:,2))+beta*sum(EXWTc(:,2)); % temporary performance measure (linear combination of expected wait and in-vehicle time)

                        if sum(TempCost(:,1)) < sum(mincost) && sum(S)==0 && sum(EXWTc(:,2)>maxwait)==0 % alternative requires less cost to cover same passenger within maximum wait time
                            if veh1st==1    % current vehicle location is included in RtPrsnt
                                mincost = TempCost(2:end,1);        % archive temporary segment travel cost
                                minroute = TempRoute(2:end,:);      % archive temporary route
                                mincumcost=TempCumCost(2:end,:);    % archive temporary cumulative segment travel cost
                            else % not included
                                mincost = TempCost(:,1);
                                minroute = TempRoute;
                                mincumcost=TempCumCost;
                            end
                            minload = TempLoad;                 % archive temporary vehicle load profile
                            minST = BST;                        % archive temporary slack time
                            minpfmcmsr = temppfmcmsr;           % archive temporary performance measure
                            minEXWT=EXWTc;                      % archive temporary expected wait time
                            minEXTT=EXTTc;                      % archive temporary expected in-vehicle time
                            minWalkT=WalkTc;                    % archive temporary walking time and other time elements
                        end
                    end
                end
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
        Aroute=zeros(size(minroute,1),7);           % prepare matrix for final route
        for m=1:size(minroute)
            Aroute(m,:)=RtInfo(minroute(m,1),2:8);  % import route information to Aroute
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