function [VEH,PAX]=DtD(Pax,numNewPax,rtlength,tmst,distcnv,dwellt,T,numOpVeh,numVeh,vehcap,vehVmph,expttc,alpha,beta,maxdelay,maxwait,D,Depots)
% % Basic factors
% % 1 unit in coordination = 1 mi
% rtlength=8.2;    % length of route (one-way)
% tmst=1; % time step: 1 sec
% ophr=4; % system operation hour (hr)
% triptime=7200; % practical one-way trip time (sec) *it can be derived from the assumed relationship with shortesttime
% dwellt=20;  % average dwell time at stops: 20 sec 
% layover=0;    % layover time at terminals
% warmupt=10;
% % warmupt=triptime*2+layover;   % time for warming up (sec) (locate vehicles en-route)
% T=3600*ophr+warmupt;    % # of time steps; simulation time= tmst*T (sec)
% numVeh=40;  % # of vehicles in fleet
% numOpVeh=numVeh; % # of operating vehicles
% % vehhdwy=720;    % vehicle headway (sec)
% vehiclespeed=7.13;    % average vehicle speed (mi/h)=(mi/3600sec)
% vehspeed=vehiclespeed/3600;  % average vehicle speed (mi/sec)
% % shortesttime=rtlength/vehspeed; % shortest one-way trip time (sec)
% % FixedStops=[1,0,0;(2:51)',0.2*(1:50)',zeros(50,1)]; % coordination of fixed stops

% nDpts=8;    % # of checkpoints between terminals
% D=nDpts+2;  % # of all checkpoints
% Depots=[(1:D)',((1:D)'-1)*(rtlength/(D-1)),zeros(D,1)];   % checkpoint information (id,x,y)
% expttc=1/vehspeed;      % expected travel time cost (pace, sec/mi)
% mvdts=vehspeed*tmst;   % moving distance per time step (mi)
% walkspeed=3.1;      % walking speed (mph)
% walklimit=0.5;      % maximum distance for walking (mi)


% % service area: rectangle
% VRange=[-0.5,0.5];  % vertical range of region (mi)
% HRange=[0,rtlength];      % horizontal range of region (mi)

% % weight for travel time
% alpha=1;    % weight for ride time
% beta=1.5;   % weight for wait time
% gamma=2;  % weight for walking time

% 1. Arrays for passenger flow per time step
numRejPax=zeros(T,1);       % number of rejected passengers
numServedPax=zeros(T,1);    % number of served passengers

% 2. Vehicle distribution on depots 
% calculate number of vehicle available for each depot
if numVeh>=D % number of vehicle is more than or equal to number of depot
    % create triangular distribution
    if mod(D,2)==1  % number of depot is odd
        maxlot=(D+1)/2;     % maximum assignable vehicle at depot
        Depdisx=[(1:maxlot)';(maxlot-1:-1:1)']; % distribution of which middle is the highest
    else            % number of depot is even
        maxlot=D/2;
        Depdisx=[(1:maxlot)';(maxlot:-1:1)'];
    end
    DepDis=[(1:D)',ones(D,1)];  % assign one vehicle per depot first
    renumVeh=numVeh-D;          % number of remaining vehicles
    DepDis(:,2)=DepDis(:,2)+round(renumVeh*Depdisx(:,1)/sum(Depdisx(:,1)));
    DepDis(round(D/2),2)=DepDis(round(D/2),2)+numVeh-sum(DepDis(:,2));
else % number of vehicle is less than number of depot
    DepDis=[(1:D)',ones(D,1)];          % create "one vehicle per depot" situation
    if mod(D,2)==mod(numVeh,2)  % (odd and odd) or (even and even)
        DepDis(1:((D-numVeh)/2),2)=0;       % delete vehicles at some first depots
        DepDis((end-(D-numVeh)/2):end,2)=0; % delete vehicles at some last depots
    else    % different (odd and even)
        DepDis(1:floor((D-numVeh)/2),2)=0;       % delete vehicles at some first depots
        DepDis((end-ceil(D-numVeh)/2):end,2)=0; % delete vehicles at some last depots
    end
end

% arrange depot to vehicle
DepotVehDist=zeros(D,1);
m=1;
for i=1:D
    for j=1:DepDis(i,2)
        DepotVehDist(m,1)=DepDis(i,1);  % depot ID for vehicle ID of m
        m=m+1;
    end
end

% 3. Vehicle information
VEH=struct;     % prepare data structure
for i=1:numVeh
    VEH(i).ID=i;        % vehicle ID
    VEH(i).q=vehcap;    % vehicle capacity
    VEH(i).v=vehVmph;   % avg vehicle speed (mph)
%     VEH(i).opIndex=0;       % operation index (1: en-route, 0: at garage)
    VEH(i).Depot=Depots(DepotVehDist(i,1),2:3);
    VEH(i).LocNow=VEH(i).Depot;     % current location: depot
    VEH(i).LocLog=[0,0];    % vehicle location log
    VEH(i).load=0;          % current load
    VEH(i).LoadLog=0;       % vehicle load log
    
    % route information: [StopID,X,Y,departure time,stop type index (3 for fixed stops),dwell time,net change of number of passenger]
    VEH(i).Rt1=[0,VEH(i).LocNow,0,0,0,0];   
    VEH(i).RT=[];           % prepare archive for routes
    VEH(i).RtVstd=zeros(1,size(VEH(i).Rt1,2));        % visited stops
    VEH(i).DistLeft=zeros(1,2);          % remained horizontal/vertical distance to next stop
    % passenger information: [PaxID,OStopID,DStopID,Boarding Index(0:arranged,1:on-board,2:processed (alighted))]
    VEH(i).Pax1=zeros(1,4);         % empty at the beginning (passengers with rightward route)
    VEH(i).PaxServed=zeros(1,4);    % prepare archive for passengers
    
    VEH(i).stayt=0;         % remaining time for vehicle to stay at node
    VEH(i).Cost1=0;         % required segment travel time (empty at beginning)
    VEH(i).CumCost1=0;      % cumulative segment travel time (empty at beginning)
        
%     for j=1:D-1
%         VEH(i).SlackT1(j,1:2)=(triptime-rtlength/vehiclespeed*3600)/(D-1)-dwellt;
%         VEH(i).SlackT1(j,3)=VRange(1,2)*2/vehspeed+dwellt;   % slack time for C-1 sections: [initial, available, usable]
%     end
%     VEH(i).SlackT1(:,4)=(1:D-1)';
    VEH(i).Extt=[0,0];            % expected travel time of arranged pax [PaxID,Extt]
    VEH(i).Exwt=[0,0];            % expected wait time of arranged pax [PaxID,Exwt]
    VEH(i).TimeV=[0,0,0,0,0,0,0,0,0];       % Time variables (PaxID,t,Drtt,Exwt0,Extt0,Exwt,Extt,Rwt,Rtt)
    VEH(i).Rtt=[0,0];             % actual travel time of arranged pax
    VEH(i).Rwt=[0,0];             % actual wait time of arranged pax
    VEH(i).WalkT1=[0,0,0,0,0];    % elements of travel time of arranged pax (id,owalkt,dwalkt,waitt,ridet)
    VEH(i).stayt=9999;              % remaining time for vehicle to stay at node (given big number means vehicle stays at depot until receiving request)
%     VEH(i).SlackArch=[0,0,0,0,0];     % archived slack time [t, Chkpnt, initial, available]
    VEH(i).idle=1;                  % vehicle idling indicator: (idle (has no route): 1, otherwise 0)
    VEH(i).PM=0;                    % performance measure
end

% SlackTemp=VEH(1).SlackT1;   % template for initial slack time setting
% CostTemp=VEH(1).Cost1;  % template for initial cost setting
% CumCostTemp=VEH(1).CumCost1;

% prepare passenger data structure
cumNumPax=zeros(T,1);   % cumulative number of passenger requests
PAX=struct('id',[],'O',[],'D',[],'hDist',[],'vDist',[],'onbrd',[],'drtt',[],'extt',[],'rtt',[],'exwt',[],'rwt',[],'btime',[],'atime',[],'veh',[]);
% IncomingPax=struct;
K=zeros(size(Pax,1),1);   % chosen vehicle for customers

% Conduct simulation
for t = 1:T
    % recall passengers regarding type distribution and list them to incoming request list
    if t==1
        cumNumPax(t,1)=numNewPax(t,1);
        IncomingPax=IdentPax_DtD(cumNumPax(t,1),0,Pax,expttc,dwellt);
        for i=1:cumNumPax(t,1)
            PAX(i)=IncomingPax(i);
        end
        a=1;                    % Starting ID of incoming passenger
        b=cumNumPax(t,1);       % Ending ID of incoming passenger
    else
        cumNumPax(t,1)=cumNumPax(t-1,1)+numNewPax(t,1);
        IncomingPax=IdentPax_DtD(cumNumPax(t,1),cumNumPax(t-1,1),Pax,expttc,dwellt);
        for i=cumNumPax(t-1,1)+1:cumNumPax(t,1)
            PAX(i)=IncomingPax(i-cumNumPax(t-1,1));
        end
        a=cumNumPax(t-1,1)+1;   % Starting ID of incoming passenger
        b=cumNumPax(t,1);       % Ending ID of incoming passenger
    end

    % vehicle dispatch information update
%     K=zeros(numNewPax(t,1),1);   % chosen route for new customers at t
    % prepare data structure of candidate route per vehicle     
    CAND=struct('Rt',[],'RtCost',[],'perform',[],'Exwt',[],'Extt',[],'Pax',[],'WalkT',[]);
    % field explanation of CAND (struct for candidate routes for each operating vehicle)
    % Rt: candidate route information []
    % RtCost: route cost information
    % perform: performance measure of candidate route
    % Exwt: expected wait time of candidate route
    % Extt: expected travel time of candidate route
    % Pax: pax assigned to candidate route
    % SlackT: slack time of candidate route

    % conduct insertion heuristic and determine the best vehicle to match
    for j=a:b
        CandRtInfo=zeros(numOpVeh,1);   % prepare array for performance measure of candidate routes
        for k=1:numOpVeh % find best vehicle to process new customer
            CAND(k)=Insert_DtD(VEH(k),PAX(j),alpha,beta,t,maxdelay,maxwait);
%             CandRtInfo(1,k)=k;                   % vehicle number
%             CandRtInfo(2,k)=size(CAND(k).Rt,1);  % length of new route (# of stops)
%             CandRtInfo(3,k)=CAND(k).perform;     % performance measure of new route
            if CAND(k).perform==Inf % route is infeasible if performance measure is undefined
                CandRtInfo(k,1)=999999; % infeasible
            else
                CandRtInfo(k,1)=CAND(k).perform; % feasible
            end
        end
        if sum(CandRtInfo(:,1)<999999)>0 % there exist some routes that can accept new customer
            [~,Kj]=min(CandRtInfo(:,1)); % find the route with shortest wait time of j
            K(j,1)=Kj;  % indicate vehicle ID that transports passenger j

            % update vehicle and route information
            VEH(Kj).Rt1=CAND(Kj).Rt;                % route information
            VEH(Kj).Cost1=CAND(Kj).RtCost(:,1);     % segment travel time: ARouteCost in Insert_DtD
            VEH(Kj).CumCost1=CAND(Kj).RtCost(:,2);  % cumulative segment travel time: ACumRouteCost in Insert_DtD
            VEH(Kj).Pax1=CAND(Kj).Pax;              % passenger information
            VEH(Kj).WalkT1=CAND(Kj).WalkT;          % walking time (including wait and in-vehicle time)
            if VEH(Kj).idle==1      % vehicle is currently idle 
                VEH(Kj).idle=0;     % no more idle as new passenger is assigned
                VEH(Kj).stayt=tmst;    % leaving current location (depot)
            end

            % archive time information to TimeV (9 columns, 2 columns for owalkt and dwalkt omitted)
            VEH(Kj).TimeV(end+1,1:3)=[j,t,PAX(j).drtt];     % Column1-3: PaxID,request time,direct travel time between OD
            for k=2:size(CAND(Kj).Exwt,1) % import exwt
                m=find(VEH(Kj).TimeV(:,1)==CAND(Kj).Exwt(k,1));
                if m>0 
                    if VEH(Kj).TimeV(m,5)==0 % initial exwt and extt is not imported yet
                        VEH(Kj).TimeV(m,4)=CAND(Kj).Exwt(k,2);  % Column4: initial exwt (exwt0)
                        VEH(Kj).TimeV(m,6)=VEH(Kj).TimeV(m,4);  % Column6: exwt to be updated (reduced in real-time as time goes)
                    else % initial exwt and extt was imported previously
                        VEH(Kj).TimeV(m,6)=CAND(Kj).Exwt(k,2);  % modified exwt after 1st match
                    end
                end
            end
            for k=2:size(CAND(Kj).Extt,1) % import extt
                m=find(VEH(Kj).TimeV(:,1)==CAND(Kj).Extt(k,1));
                if m>0
                    if VEH(Kj).TimeV(m,5)==0
                        VEH(Kj).TimeV(m,5)=CAND(Kj).Extt(k,2); % Column5: initial extt (extt0)
                        VEH(Kj).TimeV(m,7)=VEH(Kj).TimeV(m,5); % Column7: extt to be updated
                    else
                        VEH(Kj).TimeV(m,7)=CAND(Kj).Extt(k,2); % modified extt after 1st match
                    end
                end
            end
            % Column8-9: finalized real wait time and in-vehicle time
            
            VEH(Kj).PM=CAND(Kj).perform;    % performance measure increment
            VEH(Kj).Exwt=CAND(Kj).Exwt;     % expected wait time
            VEH(Kj).Extt=CAND(Kj).Extt;     % expected in-vehicle time
            VEH(Kj).Rwt=[VEH(Kj).Rwt;j,0];  % append new passenger to array tracking real wait time
            
            % update passenger information
            PAX(j).veh=Kj;  % vehicle to which passenger is assigned
            PAX(j).exwt=VEH(Kj).Exwt(VEH(Kj).Exwt(:,1)==j,2);   % import exwt
            PAX(j).extt=VEH(Kj).Extt(VEH(Kj).Extt(:,1)==j,2);   % import extt
        else % rejected (no match with any vehicle)
            PAX(j).onbrd=999; 
            numRejPax(t,1)=numRejPax(t,1)+1;
        end
    end
    
    % check feasibility (vehicle capacity and staying time)
    for i=1:numVeh
        if VEH(i).load>VEH(i).q % vehicle load exceeds capacity
            t
        end
        if VEH(i).stayt<0 % time that vehicle stays at a stop is negative
            t
        end
    end
        
    % vehicle movement according to designated routes
%     VEHI=struct;    % temporary storage for vehicle information
    for i=1:numOpVeh
%         if t>VEH(i).dispatch && VEH(i).stayt>0
        if VEH(i).stayt>0   % vehicle is staying at a stop
            [VEHI] = Dwell_DtD(VEH(i),tmst,distcnv,t);    % process vehicle information when dwelling
            VEH(i)=VEHI;    % update vehicle information
%         elseif t>VEH(i).dispatch && VEH(i).stayt==0
        elseif VEH(i).stayt==0  % vehicle is running
            [VEHI] = VehBrdAlght_DtD(VEH(i),tmst,distcnv,PAX,numServedPax,t);  % process vehicle information when running
            VEH(i)=VEHI;    % update vehicle information
        end
    end
    
%     if t==VEH(numOpVeh).dispatch+1 % time reached dispatch time of one of vehs        
%         numOpVeh=min(numVeh,numOpVeh+1);
%     end
    % error check (vehicle capacity and load)
    for i=1:numVeh
        if VEH(i).load>VEH(i).q
            t
        elseif VEH(i).load<0
            t
        end
    end
    t
end

% process simulation outputs
np=0;
MaxLoad=zeros(numVeh,1);
AvgMsr=zeros(numVeh,5);
for i=1:numVeh
    np=np+size(VEH(i).Pax1,1)+size(VEH(i).PaxServed,1)-1;
    MaxLoad(i,1)=max(VEH(i).LoadLog);
    z=0;
    for j=1:size(VEH(i).TimeV,1)
        if VEH(i).TimeV(j,6)==0 && VEH(i).TimeV(j,7)==0
            AvgMsr(i,1)=AvgMsr(i,1)+VEH(i).TimeV(j,8); % aggregate Rwt
            AvgMsr(i,2)=AvgMsr(i,2)+VEH(i).TimeV(j,9); % aggregate Rtt
            AvgMsr(i,3)=AvgMsr(i,3)+VEH(i).TimeV(j,3); % aggregate Drtt
            z=z+1;
        end
    end
    AvgMsr(i,:)=AvgMsr(i,:)/z;
    AvgMsr(i,4)=AvgMsr(i,2)/AvgMsr(i,3);
    AvgMsr(i,5)=z; % # of pax included in measure
    VEH(i).PM=beta*sum(VEH(i).TimeV(:,6))+alpha*sum(VEH(i).TimeV(:,7));
end
np/b;
np
[MaxLoad,AvgMsr];

LPK=zeros(1,9);
LPKK=zeros(1,5);
for i=1:numVeh
    LPK=[LPK;VEH(i).TimeV];
    LPKK=[LPKK;VEH(i).WalkT1];
end
LPK=sortrows(LPK,1);
LPK(LPK(:,1)==0,:)=[];
LPKK=sortrows(LPKK,1);
LPKK(LPKK(:,1)==0,:)=[];

VehOpDist=0;
for i=1:numOpVeh
    for j=2:T
        VehOpDist=VehOpDist+abs(VEH(i).LocLog(j-1,1)-VEH(i).LocLog(j,1))+abs(VEH(i).LocLog(j-1,2)-VEH(i).LocLog(j,2));
    end
end
VehOpDist

for i=1:numOpVeh
    NMN(i,1)=max(VEH(i).LoadLog);
end