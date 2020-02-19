function [CAND]=Insert_Fix(VEH,PAX,alpha,beta,gamma,C,walkspeed,rtlength,t)
% % INPUTS % %



% % OUTPUTS % %



% input vehicle and passenger information
vehcap=VEH.q;   % capacity
vehspeed=VEH.v; % vehicle speed (mph)
CAND=struct;    % prepare data structure for candidate route
if PAX.O(1,2)<PAX.D(1,2) % passenger trip is rightward
    RtPrsnt=VEH.Rt1;    % current route
    RtCost=VEH.Cost1;   % current segment travel time
    drctn=1;            % current vehicle direction
    Pax=VEH.Pax1;       % current engaged passenger
    WalkT=VEH.WalkT1;   % walking time of current engaged passenger
else % trip is leftward
    RtPrsnt=VEH.Rt2;
    RtCost=VEH.Cost2;
    drctn=2;
    Pax=VEH.Pax2;
    WalkT=VEH.WalkT2;
end

% find potential stops which passengers can reach from their O and to their D
StopMat=[floor(PAX.O(1,2)/(rtlength/(C-1))),ceil(PAX.O(1,2)/(rtlength/(C-1)));floor(PAX.D(1,2)/(rtlength/(C-1))),ceil(PAX.D(1,2)/(rtlength/(C-1)))]+1;
if drctn==2 % horiznotal flip if route is leftward
    StopMat=fliplr(StopMat); 
end

% prepare array of passenger information 
PaxInfo=[Pax;PAX.id,PAX.O(1,1),PAX.D(1,1),0,0,0]; %[PaxID,O_ID,D_ID,Boarding index,OStopID,DStopID]
numPax=size(PaxInfo,1); % number of passengers +1 (because the 1st row is zero-row)

% check if current route already passed potential O stop
if sum(RtPrsnt(:,1)==StopMat(1,1))==0 && sum(RtPrsnt(:,1)==StopMat(1,2))==0 % current route does not include neither StopMat(1,1) nor StopMat(1,2)
    PerfArray = [0,0,Inf]; % get rid of this vehicle by assigning infinite performance measure
else
    PerfArray = [0,0,99999]; % assign big number to performance measure that will be updated later
    if sum(RtPrsnt(:,1)==StopMat(1,1))>0 % current remained route includes StopMat(1,1)
        v=1; % consider both StopMat(1,1) and StopMat(1,2)
    else
        v=2; % consider only StopMat(1,2)
    end
end

% check feasibility 
Load=0; % initial vehicle load
if PerfArray(1,3)<Inf
    for i=v:2 % O stop index
        for j=1:2 % D stop index
            % O stop is different from D stop, and sequence of OD stops is consistent with vehicle direction
            if StopMat(1,i)~=StopMat(2,j) && ((StopMat(1,i)-StopMat(2,j)<0&&drctn==1)||(StopMat(1,i)-StopMat(2,j)>0&&drctn==2))
                % From StopMat(1,i) to StopMat(2,j)
                owalkt=GridDist(PAX.O(1,2:3),RtPrsnt(RtPrsnt(:,1)==StopMat(1,i),2:3))/walkspeed*3600; % walk time from real O to StopMat(1,i)
                % wait time validation
                waitt=RtPrsnt(RtPrsnt(:,1)==StopMat(1,i),4)-t-owalkt;   % wait time at StopMat(1,i): vehicle departure time - current time - walking time
                if waitt>=0
                    ivt=abs(StopMat(2,j)-StopMat(1,i))*PAX.O(1,6)+abs(StopMat(2,j)-StopMat(1,i))*(rtlength/(C-1))/vehspeed*3600; % ride time from StopMat(1,i) to StopMat(2,j)
                    dwalkt=GridDist(RtPrsnt(RtPrsnt(:,1)==StopMat(2,j),2:3),PAX.D(1,2:3))/walkspeed*3600; % walk time from StopMat(2,j) to real D
%                     Load=0;
                    tempPM = alpha*ivt + beta*waitt + gamma*(owalkt+dwalkt);    % performance measure: linear combination of in-vehicle, wait, and walking time
                    % update performance measure and passenger information
                    if PerfArray(1,3) > tempPM
                        PerfArray = [StopMat(1,i),StopMat(2,j),tempPM]; % [OStopID,DStopID,Performance measure];
                        PaxInfo(end,5)=StopMat(1,i);    % update OStopID in passenger information
                        PaxInfo(end,6)=StopMat(2,j);    % update DStopID in passenger information
                        WTs=[PAX.id,owalkt,dwalkt,waitt,ivt];   % archive calculated walking, wait, and in-vehicle time
                    end                    
                end
            else
                break % reject this O-D stop combination
            end
        end
    end
    
    % assign excessive load to vehicle to reject when no combination is chosen
    if PerfArray(1,1)==0 
        Load=999;
    end
    
    % update route information
    if Load~=999
        if drctn==VEH.drctn % vehicle direction is the same as passenger trip direction
            VehLoad = VEH.load; % start from current vehicle load
        else % if opposite
            VehLoad = 0; % start from 0
        end

        % change route information, especially for net change of stops
        re=size(RtPrsnt,1); % length of current route
        RtInfo=[(1:re)',RtPrsnt];    % route information [StopSequence,StopID,X,Y,departure time,stop type index (3 for fixed stops),dwell time,net change of number of passenger]
        
        % find sequences of O stop and D stop
        p=RtInfo(RtInfo(:,2)==PaxInfo(end,5),1);    % sequence of O stop
        RtInfo(p,8)=RtInfo(p,8)+PAX.O(1,7);         % load pax
        q=RtInfo(RtInfo(:,2)==PaxInfo(end,6),1);    % sequence of D stop
        RtInfo(q,8)=RtInfo(q,8)+PAX.D(1,7);         % unload pax        
        
%         p=1;
%         while RtInfo(p,2)~=PaxInfo(end,5)
%             p=p+1;
%         end
%         while RtInfo(p,2)~=PaxInfo(end,6)
%             p=p+1;
%         end
%         RtInfo(p,8)=RtInfo(p,8)+PAX.D(1,7); % unload pax

        % update vehicle load profile
    %     BRoute=A; % label of existing route
        Load(1,1)=VehLoad+RtInfo(1,8); % initiate load profile
    %     RtInfo(1,8)=0;
        for i=2:re % reflect net change of passenger numbers
    %         if RtInfo(i,5)>0 % i-th stop is NP or ND, change load
            Load(i,1)=Load(i-1,1)+RtInfo(i,8);
    %         elseif RtInfo(i,5)==0 % i-th stop is chkpnt, calculate net change of load
    %             BLoad(i,1)=BLoad(i-1,1)+sum(PaxInfo(PaxInfo(:,2)==RtInfo(i,2)))+sum(PaxInfo(PaxInfo(:,3)==RtInfo(i,2)));
    %         end
        end
    %       RouteCost=RtCost;
        % update travel time on sections between stops
        CumCost(1,1)=RtCost(1,1); % initiate travel time
        for i=2:size(RtCost,1)
            CumCost(i,1)=CumCost(i-1,1)+RtCost(i,1)+RtPrsnt(i,6); % accumulate travel time (previous cumulative travel cost+current travel cost+dwell time)
        end
    end
else 
    Load=999; % assign excessive load to vehicle to reject 
end

% check feasibility (vehicle capacity)
if max(Load)<=vehcap % assign final output if feasible
    Aroute=RtInfo(:,2:8);   % route information
    ARouteCost=RtCost;      % section travel time 
    ACumRouteCost=CumCost;  % cumulative section travel time
    EXWT=zeros(numPax,2);   % space for expected wait time of passengers
    EXTT=zeros(numPax,2);   % space for expected in-vehicle time of passengers
    WalkT=[WalkT;WTs];      % walking time of passengers (wait and in-vehicle time included)
    % calculate exwt and extt
    for m=2:numPax
        if PaxInfo(m,4)==0 % passenger is waiting for vehicle (not picked up yet)
            EXWT(m,1:2)=[PaxInfo(m,1),max(0,WalkT(WalkT(:,1)==PaxInfo(m,1),4)-PAX.O(1,6))];        % maximum of 0 and (waitt - dwell time at O stop)
            EXTT(m,1:2)=[PaxInfo(m,1),Aroute(Aroute(:,1)==PaxInfo(m,6),4)-t-PAX.D(1,6)-EXWT(m,2)]; % departure time of D stop - current time - dwell time at D stop - expected wait time
        elseif PaxInfo(m,4)==1 % passenger is onboard (not dropped off yet)
            EXWT(m,1:2)=[PaxInfo(m,1),0];   % exwt is zero
            EXTT(m,1:2)=[PaxInfo(m,1),Aroute(Aroute(:,1)==PaxInfo(m,6),4)-t-PAX.D(1,6)];    % departure time of D stop - current time - dwell time at D stop
        end
    end
    pfmcmsr=alpha*sum(EXTT(:,2))+beta*sum(EXWT(:,2));
    Aload=Load;
else % assign null result if capacity condition violated
    Aroute=[];
    ARouteCost=[];
    ACumRouteCost=0;
    EXTT=[0,0];
    EXWT=[0,0];
    WalkT=[0,0,0,0,0];
    pfmcmsr=inf;
    Aload=inf;
end

% arrange final outputs to candidate route data structure
CAND.Rt=Aroute;
CAND.RtCost=[ARouteCost,ACumRouteCost];
CAND.perform=pfmcmsr;
CAND.Exwt=EXWT;
CAND.Extt=EXTT;
CAND.WalkT=WalkT;
CAND.Pax=PaxInfo;

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