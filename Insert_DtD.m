function [CAND]=Insert_DtD(VEH,PAX,alpha,beta,t,maxdelay,maxwait)
% % INPUTS % %
% VEH: vehicle data structure
% PAX: passegner data structure
% alpha: weight for in-vehicle time
% beta: weight for wait time
% t: current time

% % OUTPUTS % %
% CAND: candidate route data structure


% input vehicle and passenger information
CAND=struct;    % prepare data structure for candidate route
RtPrsnt=VEH.Rt1;            % current route
% RtCost=VEH.Cost1;           % current segment travel time
RtCumCost=VEH.CumCost1;     % current cumulative segment travel time
Pax=VEH.Pax1;               % current engaged passenger
WalkT=VEH.WalkT1;           % walking time of current engaged passenger

% candidate route establishment according to current route
if size(RtPrsnt,1)==1   % if no route is assigned to vehicle (route only with current depot or stop)
    Route=[RtPrsnt;PAX.O;PAX.D];    % route only consists of OD of new passenger
    ARouteCost=1/VEH.v*3600*[0;GridDist(Route(1,2:3),Route(2,2:3));GridDist(Route(2,2:3),Route(3,2:3))];    % calculate segment travel time
    
    % calculate cumulative segment travel time
    ACumRouteCost=zeros(3,1);    % initiate array of cumulative segment travel time
    for i=2:3
        ACumRouteCost(i,1)=ARouteCost(i,1)+ACumRouteCost(i-1,1);    % cumulate segment travel time
    end
    ACumRouteCost(2:3,1)=ACumRouteCost(2:3,1)+Route(2:3,6);   % reflect dwell time
    
    % arrange final outputs to candidate route data structure
    CAND.Rt=Route;      % route information
    CAND.RtCost=[ARouteCost,ACumRouteCost];         % segment travel time information
    CAND.Exwt=[0,0;PAX.id,ARouteCost(2,1)];         % expected wait time
    CAND.Extt=[0,0;PAX.id,ACumRouteCost(3,1)-Route(3,6)-ARouteCost(2,1)];   % expected in-vehicle time
    CAND.Pax=[PAX.id,PAX.O(1,1),PAX.D(1,1),0];      % passenger information: [PaxID,O_ID,D_ID,Boarding index,OStopID,DStopID]
    CAND.perform=alpha*CAND.Exwt(2,2)+beta*CAND.Extt(2,2);  % performance measure (there is no walking time for DtD)
    CAND.WalkT=[zeros(1,5);PAX.id,0,0,CAND.Exwt(2,2),CAND.Extt(2,2)];       % wait and in-vehicle time of current engaged passenger
    Aload=[0;1;0];    % vehicle load profile
else % there exist a route
    % Calculate performance measure for existing route to compare with that of candidate
    numPaxb=size(Pax,1);    % number of passengers +1 (because the 1st row is zero-row)
    EXWTb=zeros(numPaxb,2); % prepare matrix for calculating exwt
    EXTTb=zeros(numPaxb,2); % prepare matrix for calculating extt

    for m=2:numPaxb
        if Pax(m,4)==0  % passenger is waiting for vehicle (not picked up yet)
            EXWTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,2),1)-PAX.O(1,6)];             % cumulative segment travel time to origin - dwellt at origin 
            EXTTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,3),1)-EXWTb(m,2)-PAX.D(1,6)];  % cumulative segment travel time to destination - dwellt at destination
%             EXWTb(m,2)=max(0,EXWTb(m,2)-WalkT(WalkT(:,1)==Pax(m,4),2)); % deduct walking time
%             EXTTb(m,2)=EXTTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,3),6);
        elseif Pax(m,4)==1  % passenger is onboard (not dropped off yet)
            EXTTb(m,1:2)=[Pax(m,1),RtCumCost(RtPrsnt(:,1)==Pax(m,3),1)-PAX.D(1,6)];  % cumulative segment travel time to destination - dwellt at destination
%             EXTTb(m,2)=EXTTb(m,2)-RtPrsnt(RtPrsnt(:,1)==Pax(m,3),6);
        end
    end
    pfmcmsrb=alpha*sum(EXTTb(:,2))+beta*sum(EXWTb(:,2));    % walking time is not included because there is no walking for DtD
    
    % prepare array of passenger information 
    PaxInfo=[Pax;PAX.id,PAX.O(1,1),PAX.D(1,1),0]; %[PaxID,OStopID,DStopID,Boarding index]
    numPax=size(PaxInfo,1);         % number of passengers +1 (because the 1st row is zero-row)
%     nRtChk=sum(RtPrsnt(:,5)==3);    
    
%     if size(Route,1)>0
    % prepare temporary route
    Route=[RtPrsnt;PAX.O;PAX.D];    % append new passenger OD to current route (temporary route): [StopID,X,Y,StopType,PaxID,dwellt,numPax]   
    rt=size(Route,1);               % number of stops to be considered
    RtInfo=[(1:rt)',Route];         % rt*8 matrix (stop label, Route(7))
    
    % create distance matrix
    DistMat=zeros(rt,rt);           % distance matrix between stops
    for i=1:rt
        for j=1:rt
            if i<j
                DistMat(i,j)=1/VEH.v*3600*GridDist(RtInfo(i,3:4),RtInfo(j,3:4));    % calculate distance
            elseif i>j
                DistMat(i,j)=DistMat(j,i);
            end
        end
    end
    re=size(RtPrsnt,1); % number of stops within current route
    Broute=(1:re)';     % labeling stops of existing route
    mincost = inf;      % initiate segment travel cost
    minroute = Broute;  % initiate route sequence
    
    % initiate load profile
    Bload(1,1)=VEH.load+RtInfo(1,8); % create load profile
    for i=2:re
        Bload(i,1)=Bload(i-1,1)+RtInfo(i,8);    % reflect net change of number of passenger
    end
    
    % Insertion heuristic
    for j = 1:re-1  % location index for inserting new origin
        for k = j+1:re % location index for inserting new destination
%             BST=ST;
            TempLoad = [Bload(1:j,1); Bload(j,1)+PAX.O(1,7); Bload(j+1:re,1)+PAX.O(1,7)];               % temporary load profile after adding origin
            TempRoute = [Broute(1:j,1); re+1; Broute(j+1:re,1)];                                        % temporary route after adding origin
            TempLoad = [TempLoad(1:k,1); TempLoad(k,1)+PAX.D(1,7); TempLoad(k+1:re+1,1)+PAX.D(1,7)];    % temporary load profile after adding destination
            TempRoute = [TempRoute(1:k,1); re+2; TempRoute(k+1:re+1,1)];                                % temporary route after adding origin
            
            % calculate expected wait and in-vehicle time if feasible (vehicle capacity)
            if max(TempLoad) <= VEH.q % maximum load does not exceed vehicle capacity
                rs2 = size(TempRoute,1);    % number of stops within temporary route
                TempCost=zeros(rs2,1);      % initiate segment travel time
                TempCost(1,1)=1/VEH.v*3600*GridDist(VEH.LocNow,RtInfo(TempRoute(1,1),3:4))+RtInfo(TempRoute(1,1),7);    % calculate segment travel time from current location to 1st stop
                TempCumCost=zeros(rs2,1);   % initiate segment travel time
%                 if RtInfo(TempRoute(1,1),6)==3
%                     TempCumCost(1,1)=RtInfo(TempRoute(1,1),5)-t;
%                 else
                TempCumCost(1,1)=TempCost(1,1);
%                 end
                % build temporary segment travel time and its cumulative time
                for m = 1:rs2-1
                    TempCost(m+1,1)=DistMat(TempRoute(m,1),TempRoute(m+1,1))+RtInfo(TempRoute(m+1,1),7);    % segment travel time between m-th and (m+1)-th stop + dwellt at (m+1)-th
                    TempCumCost(m+1,1)=TempCumCost(m,1)+TempCost(m+1,1);    % cumulative segment travel time before leaving (m+1)-th stop
                end
                    
                % feasibility check (maximum detour delay and maximum wait time)
                P(1:numPax)=0;  % indicator for excessive in-vehicle time
                Q(1:numPax)=0;  % indicator for excessive wait time
                for m = 1:numPax % consider all passenger
                    if PaxInfo(m,4)==0  % passenger is waiting for vehicle (not picked up yet)
                        exwt=TempCumCost(TempRoute(:,1)==RtInfo(RtInfo(:,2)==PaxInfo(m,2),1),1)-RtInfo(RtInfo(:,2)==PaxInfo(m,2),7);        % cumulative segment travel time to origin - dwellt at origin 
                        extt=TempCumCost(TempRoute(:,1)==RtInfo(RtInfo(:,2)==PaxInfo(m,3),1),1)-exwt-RtInfo(RtInfo(:,2)==PaxInfo(m,3),7);   % cumulative segment travel time to destination - dwellt at destination
                        if extt>maxdelay*VEH.TimeV(VEH.TimeV(:,1)==PaxInfo(m,1),3)  % expected in-vehicle time is longer than threshold
                            P(m)=1; % change indicator (0->1)
                        end
                        if exwt>maxwait-VEH.TimeV(VEH.TimeV(:,1)==PaxInfo(m,1),8)   % expected wait time is longer than threshold
                            Q(m)=1; % change indicator (0->1)
                        end
                    elseif PaxInfo(m,4)==1  % passenger is onboard (not dropped off yet)
                        extt=TempCumCost(TempRoute(:,1)==RtInfo(RtInfo(:,2)==PaxInfo(m,3),1),1)-RtInfo(RtInfo(:,2)==PaxInfo(m,3),7);    % cumulative segment travel time to destination - dwellt at destination
                        if extt>maxdelay*VEH.TimeV(VEH.TimeV(:,1)==PaxInfo(m,1),3)-VEH.TimeV(VEH.TimeV(:,1)==PaxInfo(m,1),9)            % expected in-vehicle time is longer than threshold
                            P(m)=1; % change indicator (0->1)
                        end
                    end
                end
                
                % save temporary route and its information if feasible
                if (sum(TempCost(:,1)) < sum(mincost)) && sum(P)+sum(Q)==0  % route requires shorter time to cover the same stops and satisfies in-vehicle and wait time threshold
                    mincost = TempCost(:,1);    % segment travel time
                    mincumcost=TempCumCost;     % cumulative segment travel time
                    minroute = TempRoute;       % route sequence
                    minload = TempLoad;         % load profile
%                     minST = BST;
                end
            end
        end
    end
    

%     else
%         mincost=inf;
%     end

    % arrange final outputs to candidate route data structure if feasible
    if mincost==inf % assign null result because of no appropriate route with new customer
        Aroute=[];
        ARouteCost=[];
        ACumRouteCost=0;
        EXTT=[0,0];
        EXWT=[0,0];
        pfmcmsr=inf;
%         AST=ST;
        Aload=inf;
    else % found best route with new customer
        Aroute=zeros(size(minroute,1),7);           % prepare matrix for final route
        for m=1:size(minroute)
            Aroute(m,:)=RtInfo(minroute(m,1),2:8);  % import route information to Aroute
        end
    
        ARouteCost = mincost;       % segment travel time
        ACumRouteCost=mincumcost;   % cumulative segment travel time
        
        % calculate expected wait and in-vehicle time
        EXWT=zeros(numPax,2);   % space for expected wait time of passengers
        EXTT=zeros(numPax,2);   % space for expected in-vehicle time of passengers
        for m=1:numPax
            if PaxInfo(m,4)==0  % passenger is waiting for vehicle (not picked up yet)
                EXWT(m+1,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,2),1)-Aroute(Aroute(:,1)==PaxInfo(m,2),6)];                % cumulative segment travel time to origin - dwellt at origin 
%                 EXWT(m+1,2)=EXWT(m+1,2)-Aroute(Aroute(:,1)==PaxInfo(m,2),6);
                EXTT(m+1,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,3),1)-EXWT(m+1,2)-Aroute(Aroute(:,1)==PaxInfo(m,3),6)];    % cumulative segment travel time to destination - dwellt at destination
%                 EXWT(m+1,2)=max(0,EXWT(m+1,2)-WalkT(WalkT(:,1)==PaxInfo(m,4),2));

%                 EXTT(m+1,2)=EXTT(m+1,2)-Aroute(Aroute(:,1)==PaxInfo(m,3),6);

            elseif PaxInfo(m,4)==1  % passenger is onboard (not dropped off yet)
                EXTT(m+1,1:2)=[PaxInfo(m,1),ACumRouteCost(Aroute(:,1)==PaxInfo(m,3),1)-Aroute(Aroute(:,1)==PaxInfo(m,3),6)];    % cumulative segment travel time to destination - dwellt at destination
%                 EXTT(m+1,2)=EXTT(m+1,2)-Aroute(Aroute(:,1)==PaxInfo(m,3),6);
            end
        end
        pfmcmsr=alpha*sum(EXTT(:,2))+beta*sum(EXWT(:,2));   % walking time is not included because there is no walking for DtD
%         AST=minST;
        Aload=minload;  % vehicle load profile
        
        % archive expected wait and in-vehicle time
        WTs=[PAX.id,0,0,EXWT(EXWT(:,1)==PAX.id,2),EXTT(EXTT(:,1)==PAX.id,2)]; %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        WalkT=[WalkT;WTs];
    end
    % end   
    % arrange final outputs to candidate route data structure
    CAND.Rt=Aroute;         % route
    CAND.RtCost=[ARouteCost,ACumRouteCost]; % segment travel time and its cumulative time
    CAND.perform=pfmcmsr-pfmcmsrb;          % increment of performance measure
    CAND.Exwt=EXWT;         % expected wait time
    CAND.Extt=EXTT;         % expected in-vehicle time
    CAND.WalkT=WalkT;       % walking time (including wait and in-vehicle time)
    CAND.Pax=PaxInfo;       % passenger information
%     CAND.SlackT=AST;
    
end

% simulation error check
if CAND.perform<0 % performance measure is negative
    t        
end

if max(Aload)<Inf
    if max(Aload)>VEH.q % vehicle load exceeds capacity
        t
    elseif Aload(end,1)>0 % passenger remain at the end of one-way trip
        t
    elseif Aload(1,1)<0 % initial load is negative
        t        
    end
end