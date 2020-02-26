function [VEH,PAX,LPK,LPKK]=FlexibleRoute(Pax,numNewPax,rtlength,tmst,triptime,distcnv,dwellt,warmupt,T,numOpVeh,numVeh,vehcap,vehhdwy,vehVmph,vehVmps,ChkPnts,expttc,walkspeed,walklimit,VRange,alpha,beta,maxback,maxwait)
% Basic factors
% 1 unit in coordination = 1 mi
% rtlength=8.2;    % length of route (one-way)
% tmst=1; % time step: 1 sec
% ophr=4; % system operation hour (hr)
% triptime=7200; % practical one-way trip time (sec) *it can be derived from the assumed relationship with shortesttime
% dwellt=20;  % average dwell time at stops: 20 sec 
% layover=0;    % layover time at terminals
% warmupt=triptime*2+layover;   % time for warming up (sec) (locate vehicles en-route)
% T=3600*ophr+warmupt;    % # of time steps; simulation time= tmst*T (sec)
% numVeh=20;  % # of vehicles in fleet
% numOpVeh=1; % # of operating vehicles
% vehhdwy=720;    % vehicle headway (sec)
% vehVmph=7.13;    % average vehicle speed (mi/h)=(mi/3600sec)
% vehVmps=vehVmph/3600;  % average vehicle speed (mi/sec)
% shortesttime=rtlength/vehVmps; % shortest one-way trip time (sec)
% % FixedStops=[1,0,0;(2:51)',0.2*(1:50)',zeros(50,1)]; % coordination of fixed stops
% 
% nstops=57;      % initial # of stops
% nChks=8;    % # of checkpoints between terminals
% C=nChks+2;  % # of all checkpoints
% ChkPnts=[(1:C)',((1:C)'-1)*(rtlength/(C-1)),zeros(C,1)];   % checkpoint information (id,x,y)
% expttc=1/vehVmps;      % expected travel time cost (pace, sec/mi)
% walkspeed=3.1;      % walking speed (mph)
% walklimit=0.5;      % maximum distance for walking (mi)
% 
% % service area: rectangle
% VRange=[-0.5,0.5];  % vertical range of region (mi)
% HRange=[0,rtlength];      % horizontal range of region (mi)
% 
% % weight for travel time
% alpha=1;    % weight for ride time
% beta=1.5;   % weight for wait time
% gamma=2;  % weight for walking time

% 1. Arrays for passenger flow per time step
numRejPax=zeros(T,1);       % number of rejected passengers
numServedPax=zeros(T,1);    % number of served passengers

% 2. Vehicle information
VEH=struct;     % prepare data structure
for i=1:numVeh
    VEH(i).ID=i;        % vehicle ID
    VEH(i).q=vehcap;    % vehicle capacity
    VEH(i).v=vehVmph;   % avg vehicle speed (mph)
    
%     VEH(i).opIndex=0;       % operation index (1: en-route, 0: at garage)
    VEH(i).LocNow=[-0.01,0];    % current location: out of route initially
    VEH(i).LocLog=[0,0];    % vehicle location log
    VEH(i).load=0;          % current load
    VEH(i).LoadLog=0;       % vehicle load log
    VEH(i).drctn=1;         % vehicle direction (1: rightward, 2: leftward)
    VEH(i).dispatch=vehhdwy*(i-1);     % time when vehicle is dispatched
    
    % route information: [StopID,X,Y,departure time,stop type index (3 for fixed stops),dwell time,net change of number of passenger]
    C=size(ChkPnts,1);    % total number of checkpoints
    VEH(i).Rt1=[ChkPnts,zeros(C,1),3*ones(C,1),dwellt*ones(C,1),zeros(C,1)]; % rightward route 
    VEH(i).Rt2=flipud(VEH(i).Rt1);  % leftward route: flip rightward route
    VEH(i).RT=[];           % prepare archive for routes
%     VEH(i).rtseq=0;                 % current sequence of route
    VEH(i).RtVstd=zeros(1,size(VEH(i).Rt1,2));        % visited stops
    VEH(i).DistLeft=zeros(1,2);          % remained horizontal/vertical distance to next stop
    VEH(i).Pax1=zeros(1,4);               % arranged pax: [PaxID,OStopID,DStopID,Boarding Index(0:arranged,1:on-board,2:processed (alighted))]
    VEH(i).Pax2=VEH(i).Pax1;        % pax when drctn==2
    VEH(i).PaxServed=zeros(1,4);  
%     VEH(i).rmnCP=C;                 % # of remained checkpoints to stop
    
    % segment travel time and cumulative segment travel time for checkpoints
    VEH(i).Cost1=[0;ones(C-1,1)*triptime/(C-1)];    % segment travel time on rightward route
    VEH(i).CumCost1=zeros(C,1); % prepare array for cumulative segment travel time
    for j=2:C
        VEH(i).CumCost1(j,1)=VEH(i).CumCost1(j-1,1)+VEH(i).Cost1(j,1);  % add cumulative segment travel time to (j-1)-th checkpoint and segment travel time between (j-1)-th and j-th checkpoint 
    end
    VEH(i).Cost2=VEH(i).Cost1;          % segment travel time on leftward route (same was rightward)
    VEH(i).CumCost2=VEH(i).CumCost1;    % cumulative segment travel time on leftward route (same was rightward)
    
    % departure time of checkpoints
    for j=1:C
        VEH(i).Rt1(j,4)=triptime/(C-1)*(j-1)+VEH(i).dispatch;  % timetable at checkpoints on rightward route
    end
    VEH(i).Rt2(1,4)=VEH(i).Rt1(C,4);    % departing end of rightward route is the same as departing start of leftward route
    for j=2:C
        VEH(i).Rt2(j,4)=triptime/(C-1)*(j-1)+VEH(i).Rt1(C,4);  % timetable at checkpoints on leftward route
    end
    % calculate slack time [initial slack time, availale slack time, usable slack time, Checkpoint ID, backtracking distance per checkpoint]
    for j=1:C-1
        VEH(i).SlackT1(j,1:2)=(triptime-rtlength/vehVmph*3600)/(C-1)-dwellt;    % initial slack time (segment travel time - dwellt) = available slack time at the beginning
        VEH(i).SlackT1(j,3:5)=[VRange(1,2)*2/vehVmps+dwellt,j,maxback];         % usable slack time, checkpoint ID, backtracking distance per checkpoint segment
    end
    VEH(i).SlackT2=flipud(VEH(i).SlackT1);  % slack time for leftward route
    
    VEH(i).stayt=0;               % remained time for vehicle to stay at node
    VEH(i).Extt=[0,0];            % expected travel time of arranged pax [PaxID,Extt]
    VEH(i).Exwt=[0,0];            % expected wait time of arranged pax [PaxID,Exwt]
    VEH(i).TimeV=[0,0,0,0,0,0,0,0,0];       % Time variables (PaxID,t,Drtt,Exwt0,Extt0,Exwt,Extt,Rwt,Rtt)
    VEH(i).Rtt=[0,0];             % actual travel time of arranged pax
    VEH(i).Rwt=[0,0];             % actual wait time of arranged pax
    VEH(i).WalkT1=[0,0,0,0,0];    % elements of travel time of arranged pax (id,owalkt,dwalkt,waitt,ridet)
    VEH(i).WalkT2=[0,0,0,0,0];    
    VEH(i).SlackArch=[0,0,0,0,0];   % archived slack time [t, Chkpnt, initial, available]
    VEH(i).PM=0;                    % performance measure
end

CostTemp=VEH(1).Cost1;          % template for initial cost setting
CumCostTemp=VEH(1).CumCost1;    % template for initial cumcost setting
SlackTemp=VEH(1).SlackT1;       % template for initial slack time setting

% prepare passenger data structure
cumNumPax=zeros(T,1);   % cumulative number of passenger requests
PAX=struct('id',[],'type',[],'O',[],'D',[],'hDist',[],'vDist',[],'drctn',[],'onbrd',[],'drtt',[],'extt',[],'rtt',[],'exwt',[],'rwt',[],'btime',[],'atime',[],'veh',[],'OO',[],'LocLog',[],'DD',[],'AddWalkT',[]);
PaxRe=zeros(1,5);       % record primarily rejected pax
K=zeros(size(Pax,1),1); % array for chosen vehicle for customers

% Conduct simulation
for t = 1:T
    % vehicle dispatch
    if t==VEH(numOpVeh).dispatch+1  % simulation reached dispatch time of one of vehicles        
        VEH(numOpVeh).LocNow=[0,0]; % vehicle is sent to starting terminal
        VEH(numOpVeh).stayt=dwellt; % vehicle stays for dwell time at starting terminal
%         VEH(numOpVeh).rmnCP=VEH(numOpVeh).rmnCP-1;  
    end
    
    % simulate after warming-up period
    if t>warmupt 
        % recall passengers and list them to incoming request list
        cumNumPax(t,1)=cumNumPax(t-1,1)+numNewPax(t-warmupt,1); % aggregate cumulative number of passengers
        % process existing passenger information to passenger data structure
        IncomingPax=IdentPax_Flex(cumNumPax(t,1),cumNumPax(t-1,1),Pax,expttc,dwellt,ChkPnts,walklimit);
        for i=cumNumPax(t-1,1)+1:cumNumPax(t,1)
            PAX(i)=IncomingPax(i-cumNumPax(t-1,1)); % match passenger data structure
        end
    
        % prepare data structure of candidate route per vehicle
%         K=zeros(numNewPax(t,1),2);   % chosen route for new customers at t
        CAND=struct('Rt',[],'RtCost',[],'perform',[],'Exwt',[],'Extt',[],'Pax',[],'SlackT',[],'WalkT',[]);
        % field explanation of CAND (struct for candidate routes for each operating vehicle)
        % Rt: candidate route information []
        % RtCost: route cost information
        % perform: performance measure of candidate route
        % Exwt: expected wait time of candidate route
        % Extt: expected travel time of candidate route
        % Pax: pax assigned to candidate route
        % SlackT: slack time of candidate route

        a=cumNumPax(t-1,1)+1;   % Starting ID of incoming pax
        b=cumNumPax(t,1);       % Ending ID of incoming pax
        
        % conduct insertion heuristic and determine the best vehicle to match
        for j=a:b
            CandRtInfo=zeros(2*numOpVeh,1); % prepare array for performance measure of candidate routes
            % 1) insertion heuristic with original OD
            for k=1:numOpVeh % find best vehicle to process new customer
                % insertion heuristic
                CAND(k)=Insert_Flex(VEH(k),PAX(j),alpha,beta,walkspeed,C,t,maxwait);    
                CandRtInfo(k,1)=CAND(k).perform;    % assign performance measure
            end
            
            % 2) insertion heuristic with future intersection between existing vehicle route and potential passenger trajectory
            OAvailSeg2=zeros(1,15); % prepare array for information of segment potentially approachable from passenger origin
            DAvailSeg2=zeros(1,15); % prepare array for information of segment potentially approachable from passenger destination
            for k=1:numOpVeh
                CANDx=struct('Rt',[],'RtCost',[],'perform',[],'Exwt',[],'Extt',[],'Pax',[],'SlackT',[],'WalkT',[]); % data structure for candidate routes from 2nd insertion heuristic
                % derive available segments from passenger origin and destination
                [OAvailSeg,DAvailSeg]=PaxApproach(VEH(k),PAX(j),ChkPnts,walklimit,walkspeed,t);
                if PAX(j).type>=3 && size(OAvailSeg,1)>0    % pickup point of passenger is non-regular
                    OAvailSeg2=[OAvailSeg2;OAvailSeg];      % augment available segment for origin
                end
                if mod(PAX(j).type,2)==0 && size(DAvailSeg,1)>0 % drop-off point of passenger is non-regular
                    DAvailSeg2=[DAvailSeg2;DAvailSeg];          % augment available segment for destination
                end
                CandPfrm=Inf;   % prepare array for performance measure from using segments 
                if PAX(j).type==3 && size(OAvailSeg,1)>0    % passenger type is NPD and there is feasible segment
                    for m=1:size(OAvailSeg,1)
                        CANDx(m)=EvalSeg(VEH(k),PAX(j),alpha,beta,C,t,OAvailSeg(m,:),zeros(1,15),walkspeed,maxwait);  % insertion heuristic with approachable segment of origin
                        CandPfrm(m,1)=CANDx(m).perform; % record performance measure
                    end
                elseif PAX(j).type==2 && size(DAvailSeg,1)>0    % passenger type is PND and there is feasible segment
                    for m=1:size(DAvailSeg,1)
                        CANDx(m)=EvalSeg(VEH(k),PAX(j),alpha,beta,C,t,zeros(1,15),DAvailSeg(m,:),walkspeed,maxwait);  % insertion heuristic with approachable segment of destination
                        CandPfrm(m,1)=CANDx(m).perform; % record performance measure
                    end
                elseif PAX(j).type==4 && size(OAvailSeg,1)>0 && size(DAvailSeg,1)>0 % passenger type is PND and there are feasible segments
                    for m=1:size(OAvailSeg,1)
                        for n=1:size(DAvailSeg,1)
                            CANDx((m-1)*size(DAvailSeg,1)+n)=EvalSeg(VEH(k),PAX(j),alpha,beta,C,t,OAvailSeg(m,:),DAvailSeg(n,:),walkspeed,maxwait);   % insertion heuristic with approachable segments
                            CandPfrm((m-1)*size(DAvailSeg,1)+n,1)=CANDx((m-1)*size(DAvailSeg,1)+n).perform; % record performance measure
                        end
                    end
                end
                [mpfm,mb]=min(CandPfrm);        % find candidate route with smallest impact
                CAND(k+numOpVeh)=CANDx(mb);     % record candidate route information as final candidate
                CandRtInfo(k+numOpVeh,1)=mpfm;  % record performance measure of candidate route information
            end
            
            [Kmin,Kjx]=min(CandRtInfo); % find final candidate route with smallest impact
            if Kjx>numOpVeh     % derive vehicle ID from row index
                Kj=Kjx-numOpVeh;
            else
                Kj=Kjx;
            end
            K(j,1:2)=[Kj,Kmin]; % record ID and lowest performance measure 
            
            if Kmin<Inf % there exist some routes that can accept new customer
                % update VEH, PAX, ROUTE
                if PAX(j).drctn==1
                    VEH(Kj).Rt1=CAND(Kjx).Rt;               % route information 
                    VEH(Kj).Cost1=CAND(Kjx).RtCost(:,1);    % segment travel time: ARouteCost in Insert_Flex
                    VEH(Kj).CumCost1=CAND(Kjx).RtCost(:,2); % cumulative segment travel time: ACumRouteCost in Insert_Flex
                    VEH(Kj).SlackT1=CAND(Kjx).SlackT;       % slack time
                    VEH(Kj).Pax1=CAND(Kjx).Pax;             % passenger infomation
                    VEH(Kj).WalkT1=CAND(Kjx).WalkT;         % walking time (including wait and in-vehicle time)
                else
                    VEH(Kj).Rt2=CAND(Kjx).Rt;
                    VEH(Kj).Cost2=CAND(Kjx).RtCost(:,1);
                    VEH(Kj).CumCost2=CAND(Kjx).RtCost(:,2);
                    VEH(Kj).SlackT2=CAND(Kjx).SlackT;
                    VEH(Kj).Pax2=CAND(Kjx).Pax;
                    VEH(Kj).WalkT2=CAND(Kjx).WalkT;
                end

                % archive time information to TimeV (9 columns, 2 columns for owalkt and dwalkt omitted)
                VEH(Kj).TimeV(end+1,1:3)=[j,t,PAX(j).drtt];     % Column1-3: PaxID,request time,direct travel time between OD
                for k=2:size(CAND(Kjx).Exwt,1) % import exwt
                    m=find(VEH(Kj).TimeV(:,1)==CAND(Kjx).Exwt(k,1));
                    if m>0 
                        if VEH(Kj).TimeV(m,5)==0 % initial exwt and extt is not imported yet
                            VEH(Kj).TimeV(m,4)=CAND(Kjx).Exwt(k,2);  % Column4: initial exwt (exwt0)
                            VEH(Kj).TimeV(m,6)=VEH(Kj).TimeV(m,4);  % Column6: exwt to be updated (reduced in real-time as time goes)
                        else % initial exwt and extt was imported previously
                            VEH(Kj).TimeV(m,6)=CAND(Kjx).Exwt(k,2);  % modified exwt after 1st match
                        end
                    end
                end
                for k=2:size(CAND(Kjx).Extt,1) % import extt
                    m=find(VEH(Kj).TimeV(:,1)==CAND(Kjx).Extt(k,1));
                    if m>0
                        if VEH(Kj).TimeV(m,5)==0
                            VEH(Kj).TimeV(m,5)=CAND(Kjx).Extt(k,2); % Column5: initial extt (extt0)
                            VEH(Kj).TimeV(m,7)=VEH(Kj).TimeV(m,5); % Column7: extt to be updated
                        else
                            VEH(Kj).TimeV(m,7)=CAND(Kjx).Extt(k,2); % modified extt after 1st match
                        end
                    end
                end
                % Column8-9: finalized real wait time and in-vehicle time
                
                VEH(Kj).PM=CAND(Kjx).perform;    % performance measure increment
                VEH(Kj).Exwt=CAND(Kjx).Exwt;     % expected wait time
                VEH(Kj).Extt=CAND(Kjx).Extt;     % expected in-vehicle time
                VEH(Kj).Rwt=[VEH(Kj).Rwt;j,-CAND(Kjx).WalkT(CAND(Kjx).WalkT(:,1)==j,2)];  % append new passenger to array tracking real wait time

                % update passenger information
                PAX(j).veh=Kj;  % vehicle to which passenger is assigned
                PAX(j).exwt=VEH(Kj).Exwt(VEH(Kj).Exwt(:,1)==j,2);   % import exwt
                PAX(j).extt=VEH(Kj).Extt(VEH(Kj).Extt(:,1)==j,2);   % import extt
            else % rejected (no match with any vehicle, type=2,3,4)
                PAX(j).onbrd=999;   % rejection index
                
                % prepare to adjust origin (walk toward nearest checkpoint)
                PAX(j).OO=PAX(j).O;         % archive original origin (OO)
                COO(:,1)=abs(ChkPnts(:,2)-PAX(j).OO(1,2));  % horizontal distance between OO and checkpoints
                [~,nrstCO]=min(COO);      % identify the nearest chkpnt
                PAX(j).DD=PAX(j).D;         % archive original destination (DD)
                CDD(:,1)=abs(ChkPnts(:,2)-PAX(j).DD(1,2));  % horizontal distance between DD and checkpoints
                [~,nrstCD]=min(CDD);      % identify the nearest chkpnt
                
                % record rejected passenger for reconsideration
                PaxRe=[PaxRe;t,PAX(j).id,0,nrstCO,nrstCD];  % record rejected pax [rejected time,passenger Id,reaccetped time,checkpoint nearest to origin,checkpoint nearest to destination]
                numRejPax(t,1)=numRejPax(t,1)+1;            % increase number of rejected passenger
            end
        end
        
        % determine when to finally reject
        for j=2:size(PaxRe,1)
            if t-PaxRe(j,1)>900 && PaxRe(j,3)==0    % passenger was rejected 900 time steps before and still on list
                PaxRe(j,3)=-999;    % indicate final rejection
            end
        end
        
        % resurrection of rejected passenger
        KK=zeros(size(PaxRe,1),2);  % array for chosen vehicle for rejected passenger
        for j=2:size(PaxRe,1)
            if PAX(PaxRe(j,2)).type>=2 && PaxRe(j,3)==0 && PaxRe(j,1)~=t && mod(t-PaxRe(j,1),30)==0 % passenger has at least one non-regular stop,was not accepted, and was rejected 30*n time step before
                [PAX(PaxRe(j,2)).O(1,2:3),~,~,treqO]=Navigate(ChkPnts(PaxRe(j,4),2:3),PAX(PaxRe(j,2)).O(1,2:3),walkspeed,tmst*30,1);    % update adjusted origin
                PAX(PaxRe(j,2)).AddWalkT(1,1)=PAX(PaxRe(j,2)).AddWalkT(1,1)+min(30,treqO);  % update additional walking time to reach adjusted origin from true origin
                PAX(PaxRe(j,2)).LocLog=[PAX(PaxRe(j,2)).LocLog;PAX(PaxRe(j,2)).O(1,2:3)];   % record passenger location while walking (every 30 time steps)
                [PAX(PaxRe(j,2)).D(1,2:3),~,~,treqD]=Navigate(ChkPnts(PaxRe(j,5),2:3),PAX(PaxRe(j,2)).D(1,2:3),walkspeed,tmst*30,1);    % update adjusted destination
                PAX(PaxRe(j,2)).AddWalkT(1,2)=PAX(PaxRe(j,2)).AddWalkT(1,2)+min(30,treqD);  % update additional walking time to reach true destination from adjusted destination
                
                % redo insertion heuristic
                CandRtInfo=zeros(2*numOpVeh,1); % prepare array for performance measure of candidate routes
                % 1) insertion heuristic with original OD
                for k=1:numOpVeh % find best vehicle to process new customer
                    % insertion heuristic
                    CAND(k)=Insert_Flex(VEH(k),PAX(PaxRe(j,2)),alpha,beta,walkspeed,C,t,maxwait);    
                    CandRtInfo(k,1)=CAND(k).perform;    % assign performance measure
                end
                
                % 2) insertion heuristic with future intersection between existing vehicle route and potential passenger trajectory
                OAvailSeg2=zeros(1,15); % prepare array for information of segment potentially approachable from passenger origin
                DAvailSeg2=zeros(1,15); % prepare array for information of segment potentially approachable from passenger destination
                for k=1:numOpVeh
                    CANDx=struct('Rt',[],'RtCost',[],'perform',[],'Exwt',[],'Extt',[],'Pax',[],'SlackT',[],'WalkT',[]); % data structure for candidate routes from 2nd insertion heuristic
                    % derive available segments from passenger origin and destination
                    [OAvailSeg,DAvailSeg]=PaxApproach(VEH(k),PAX(PaxRe(j,2)),ChkPnts,walklimit,walkspeed,t);
                    if PAX(PaxRe(j,2)).type>=3 && size(OAvailSeg,1)>0    % pickup point of passenger is non-regular
                        OAvailSeg2=[OAvailSeg2;OAvailSeg];      % augment available segment for origin
                    end
                    if mod(PAX(PaxRe(j,2)).type,2)==0 && size(DAvailSeg,1)>0 % drop-off point of passenger is non-regular
                        DAvailSeg2=[DAvailSeg2;DAvailSeg];          % augment available segment for destination
                    end
                    CandPfrm=Inf;   % prepare array for performance measure from using segments 
                    
                    if PAX(PaxRe(j,2)).type==3 && size(OAvailSeg,1)>0    % passenger type is NPD and there is feasible segment
                        for m=1:size(OAvailSeg,1)
                            CANDx(m)=EvalSeg(VEH(k),PAX(PaxRe(j,2)),alpha,beta,C,t,OAvailSeg(m,:),zeros(1,15),walkspeed,maxwait);  % insertion heuristic with approachable segment of origin
                            CandPfrm(m,1)=CANDx(m).perform; % record performance measure
                        end
                    elseif PAX(PaxRe(j,2)).type==2 && size(DAvailSeg,1)>0    % passenger type is PND and there is feasible segment
                        for m=1:size(DAvailSeg,1)
                            CANDx(m)=EvalSeg(VEH(k),PAX(PaxRe(j,2)),alpha,beta,C,t,zeros(1,15),DAvailSeg(m,:),walkspeed,maxwait);  % insertion heuristic with approachable segment of destination
                            CandPfrm(m,1)=CANDx(m).perform; % record performance measure
                        end
                    elseif PAX(PaxRe(j,2)).type==4 && size(OAvailSeg,1)>0 && size(DAvailSeg,1)>0 % passenger type is PND and there are feasible segments
                        for m=1:size(OAvailSeg,1)
                            for n=1:size(DAvailSeg,1)
                                CANDx((m-1)*size(DAvailSeg,1)+n)=EvalSeg(VEH(k),PAX(PaxRe(j,2)),alpha,beta,C,t,OAvailSeg(m,:),DAvailSeg(n,:),walkspeed,maxwait);   % insertion heuristic with approachable segments
                                CandPfrm((m-1)*size(DAvailSeg,1)+n,1)=CANDx((m-1)*size(DAvailSeg,1)+n).perform; % record performance measure
                            end
                        end
                    end
                                        
                    [mpfm,mb]=min(CandPfrm);        % find candidate route with smallest impact
                    CAND(k+numOpVeh)=CANDx(mb);     % record candidate route information as final candidate
                    CandRtInfo(k+numOpVeh,1)=mpfm;  % record performance measure of candidate route information
                end

                [KKmin,KKjx]=min(CandRtInfo); % find final candidate route with smallest impact
                if KKjx>numOpVeh     % derive vehicle ID from row index
                    KKj=KKjx-numOpVeh;
                else
                    KKj=KKjx;
                end
                KK(j,1:2)=[KKj,KKmin]; % record ID and lowest performance measure 

                if KKmin<Inf % there exist some routes that can accept new customer
                    % update VEH, PAX, ROUTE
                    if PAX(PaxRe(j,2)).drctn==1
                        VEH(KKj).Rt1=CAND(KKjx).Rt;               % route information 
                        VEH(KKj).Cost1=CAND(KKjx).RtCost(:,1);    % segment travel time: ARouteCost in Insert_Flex
                        VEH(KKj).CumCost1=CAND(KKjx).RtCost(:,2); % cumulative segment travel time: ACumRouteCost in Insert_Flex
                        VEH(KKj).SlackT1=CAND(KKjx).SlackT;       % slack time
                        VEH(KKj).Pax1=CAND(KKjx).Pax;             % passenger infomation
                        VEH(KKj).WalkT1=CAND(KKjx).WalkT;         % walking time (including wait and in-vehicle time)
                    else
                        VEH(KKj).Rt2=CAND(KKjx).Rt;
                        VEH(KKj).Cost2=CAND(KKjx).RtCost(:,1);
                        VEH(KKj).CumCost2=CAND(KKjx).RtCost(:,2);
                        VEH(KKj).SlackT2=CAND(KKjx).SlackT;
                        VEH(KKj).Pax2=CAND(KKjx).Pax;
                        VEH(KKj).WalkT2=CAND(KKjx).WalkT;
                    end

                    % archive time information to TimeV (9 columns, 2 columns for owalkt and dwalkt omitted)
                    VEH(KKj).TimeV(end+1,1:3)=[PaxRe(j,2),t,PAX(PaxRe(j,2)).drtt];     % Column1-3: PaxID,request time,direct travel time between OD
                    for k=2:size(CAND(KKjx).Exwt,1) % import exwt
                        m=find(VEH(KKj).TimeV(:,1)==CAND(KKjx).Exwt(k,1));
                        if m>0 
                            if VEH(KKj).TimeV(m,5)==0 % initial exwt and extt is not imported yet
                                VEH(KKj).TimeV(m,4)=CAND(KKjx).Exwt(k,2);  % Column4: initial exwt (exwt0)
                                VEH(KKj).TimeV(m,6)=VEH(KKj).TimeV(m,4);  % Column6: exwt to be updated (reduced in real-time as time goes)
                            else % initial exwt and extt was imported previously
                                VEH(KKj).TimeV(m,6)=CAND(KKjx).Exwt(k,2);  % modified exwt after 1st match
                            end
                        end
                    end
                    for k=2:size(CAND(KKjx).Extt,1) % import extt
                        m=find(VEH(KKj).TimeV(:,1)==CAND(KKjx).Extt(k,1));
                        if m>0
                            if VEH(KKj).TimeV(m,5)==0
                                VEH(KKj).TimeV(m,5)=CAND(KKjx).Extt(k,2); % Column5: initial extt (extt0)
                                VEH(KKj).TimeV(m,7)=VEH(KKj).TimeV(m,5); % Column7: extt to be updated
                            else
                                VEH(KKj).TimeV(m,7)=CAND(KKjx).Extt(k,2); % modified extt after 1st match
                            end
                        end
                    end
                    % Column8-9: finalized real wait time and in-vehicle time
                    
                    VEH(KKj).PM=CAND(KKjx).perform;    % performance measure increment
                    VEH(KKj).Exwt=CAND(KKjx).Exwt;     % expected wait time
                    VEH(KKj).Extt=CAND(KKjx).Extt;     % expected in-vehicle time
                    VEH(KKj).Rwt=[VEH(KKj).Rwt;PaxRe(j,2),-CAND(KKjx).WalkT(CAND(KKjx).WalkT(:,1)==PaxRe(j,2),2)];  % append new passenger to array tracking real wait time

                    % update passenger information
                    PAX(PaxRe(j,2)).veh=KKj;  % vehicle to which passenger is assigned
                    PAX(PaxRe(j,2)).exwt=VEH(KKj).Exwt(VEH(KKj).Exwt(:,1)==PaxRe(j,2),2);   % import exwt
                    PAX(PaxRe(j,2)).extt=VEH(KKj).Extt(VEH(KKj).Extt(:,1)==PaxRe(j,2),2);   % import extt

                    PAX(PaxRe(j,2)).onbrd=99;   % update rejection indicator (999->99)
                    PaxRe(j,3)=t;               % record reaccepted time
                end
            end
        end
    end
    
    % vehicle movement according to designated routes
    for i=1:numOpVeh
        if t>VEH(i).dispatch && VEH(i).stayt>0  % vehicle is operating and staying at a stop      
            [VEHI] = Dwell_Flex(VEH(i),tmst,distcnv,C,t);   % process vehicle information when dwelling
            VEH(i)=VEHI;    % update vehicle information
        elseif t>VEH(i).dispatch && VEH(i).stayt==0 % vehicle is operating and running
            [VEHI,PAX,numServedPax] = VehBrdAlght_Flex(VEH(i),tmst,t,distcnv,PAX,numServedPax,C,ChkPnts,dwellt,triptime,SlackTemp,CostTemp,CumCostTemp);    % process vehicle information when running
            VEH(i)=VEHI;    % update vehicle information
        end
    end
    
    % dispatch a vehicle when time reaches its dispatch time (for warming-up)
    if t==VEH(numOpVeh).dispatch+1     
        numOpVeh=min(numVeh,numOpVeh+1);
    end
    
    % error check (vehicle capacity and load)
    for i=1:numVeh
        if VEH(i).load>VEH(i).q
            t
        elseif VEH(i).load<0
            t
        end
    end
end

np=0;
MaxLoad=zeros(numVeh,1);
AvgMsr=zeros(numVeh,5);
for i=1:numVeh
    np=np+size(VEH(i).Pax1,1)+size(VEH(i).Pax2,1)+size(VEH(i).PaxServed,1)-3;
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
    LPKK=[LPKK;VEH(i).WalkT1;VEH(i).WalkT2];
end
LPK=sortrows(LPK,1);
LPK(LPK(:,1)==0,:)=[];
LPK(:,8)=max(0,LPK(:,8));
LPKK=sortrows(LPKK,1);
LPKK(LPKK(:,1)==0,:)=[];
for i=1:size(LPKK,1)
    LPKK(i,2:3)=LPKK(i,2:3)+PAX(LPKK(i,1)).AddWalkT;
end

TypDist=zeros(1,4);
TypDist2=zeros(1,4);
for i=1:b
    if PAX(i).type==1
        TypDist(1)=TypDist(1)+1;
        if PAX(i).veh==0
            TypDist2(1)=TypDist2(1)+1;
        end
    elseif PAX(i).type==2
        TypDist(2)=TypDist(2)+1;
        if PAX(i).veh==0
            TypDist2(2)=TypDist2(2)+1;
        end
    elseif PAX(i).type==3
        TypDist(3)=TypDist(3)+1;
        if PAX(i).veh==0
            TypDist2(3)=TypDist2(3)+1;
        end
    elseif PAX(i).type==4
        TypDist(4)=TypDist(4)+1;
        if PAX(i).veh==0
            TypDist2(4)=TypDist2(4)+1;
        end
    end
end

VehOpDist=0;
for i=1:numOpVeh
    for j=warmupt+1:T
        VehOpDist=VehOpDist+abs(VEH(i).LocLog(j-1,1)-VEH(i).LocLog(j,1))+abs(VEH(i).LocLog(j-1,2)-VEH(i).LocLog(j,2));
    end
end
VehOpDist