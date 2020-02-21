function [VEH,PAX]=FixedRoute(Pax,numNewPax,rtlength,tmst,triptime,distcnv,dwellt,warmupt,T,numOpVeh,numVeh,vehcap,vehhdwy,vehVmph,vehVmps,Stops,expttc,walkspeed,walklimit,VRange,HRange,alpha,beta,gamma)
% % Basic factors
% % 1 unit in coordination = 1 mi
% rtlength=8.2;    % length of route (one-way)
% tmst=1; % time step: 1 sec
% ophr=1; % system operation hour (hr)
% % triptime=7200; % practical one-way trip time (sec) *it can be derived from the assumed relationship with shortesttime
% triptime=5280;
% dwellt=20;  % average dwell time at stops: 20 sec 
% layover=0;    % layover time at terminals
% warmupt=triptime*2+layover;   % time for warming up (sec) (locate vehicles en-route)
% T=3600*ophr+warmupt;    % # of time steps; simulation time= tmst*T (sec)
% % numVeh=15;  % # of vehicles in fleet
% numOpVeh=1; % # of operating vehicles
% % vehhdwy=720;    % vehicle headway (sec)
% numVeh=6;
% vehhdwy=1800;
% vehiclespeed=7.13;    % average vehicle speed (mi/h)=(mi/3600sec)
% vehspeed=vehiclespeed/3600;  % average vehicle speed (mi/sec)
% shortesttime=rtlength/vehspeed; % shortest one-way trip time (sec)
% % FixedStops=[1,0,0;(2:51)',0.2*(1:50)',zeros(50,1)]; % coordination of fixed stops
% 
% nStops=31;    % # of checkpoints between terminals
% C=nStops+2;  % # of all checkpoints
% Stops=[(1:C)',((1:C)'-1)*(rtlength/(C-1)),zeros(C,1)];   % checkpoint information (id,x,y), spacings are even
% expttc=1/vehspeed;      % expected travel time cost (pace, sec/mi)
% % mvdts=vehspeed*tmst;   % moving distance per time step (mi)
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
    VEH(i).dispatch=vehhdwy*(i-1);      % time when vehicle is dispatched
    
    % route information: [StopID,X,Y,departure time,stop type index (3 for fixed stops),dwell time,net change of number of passenger]
    S=size(Stops,1);    % total number of stops
    VEH(i).Rt1=[Stops,zeros(S,1),3*ones(S,1),dwellt*ones(S,1),zeros(S,1)]; % rightward route 
    VEH(i).Rt2=flipud(VEH(i).Rt1);  % leftward route: flip rightward route
    VEH(i).RT=[];           % prepare archive for routes
%     VEH(i).rtseq=0;                 % current sequence of route
    VEH(i).RtVstd=zeros(1,size(VEH(i).Rt1,2));        % visited stops
    % departure time for stops
    for j=1:S
        VEH(i).Rt1(j,4)=(j-1)*rtlength/(S-1)/(VEH(i).v/3600)+j*dwellt+VEH(i).dispatch;  % timetable for stops (Rt1)
    end
    VEH(i).Rt2(1,4)=VEH(i).Rt1(S,4);
    for j=2:S
        VEH(i).Rt2(j,4)=(j-1)*rtlength/(S-1)/(VEH(i).v/3600)+j*dwellt+VEH(i).Rt1(S,4);  % timetable for stops (Rt2)
    end
    VEH(i).DistLeft=zeros(1,2);     % remained horizontal and vertical distance to next stop
    
    % arranged pax: [PaxID,OStopID,DStopID,Boarding Index(0:arranged,1:on-board,2:processed (alighted))]
    VEH(i).Pax1=zeros(1,6);     % empty at the beginning (passengers with rightward route)
    VEH(i).Pax2=VEH(i).Pax1;    % empty at the beginning (passengers with leftward route)
    VEH(i).PaxServed=zeros(1,6);    % prepare archive for passengers
%     VEH(i).rmnCP=C;                 % # of remained checkpoints to stop
    
    VEH(i).Cost1=[0;ones(S-1,1)*rtlength/(S-1)/(VEH(i).v/3600)];    % required segment travel time on current route
    VEH(i).Cost2=VEH(i).Cost1;  % required costs is the same for both direction if stops are evenly distributed
    
    % 
%     for j=1:C-1
%         VEH(i).SlackT1(j,1:2)=(triptime-rtlength/vehiclespeed*3600)/(C-1)-dwellt;
%         VEH(i).SlackT1(j,3)=VRange(1,2)*2/vehspeed+dwellt;   % slack time for C-1 sections: [initial, available, usable]
%         VEH(i).SlackT1(j,1:3)=0;
%     end
%     VEH(i).SlackT1(:,4)=(1:C-1)';
%     VEH(i).SlackT2=flipud(VEH(i).SlackT1);
    VEH(i).Extt=[0,0];              % expected travel time of arranged pax [PaxID,Extt]
    VEH(i).Exwt=[0,0];              % expected wait time of arranged pax [PaxID,Exwt]
    VEH(i).Rtt=[0,0];             % actual travel time of arranged pax
    VEH(i).Rwt=[0,0];             % actual wait time of arranged pax
    VEH(i).WalkT1=[0,0,0,0,0];           % elements of travel time of arranged pax (id,owalkt,dwalkt,waitt,ridet)
    VEH(i).WalkT2=[0,0,0,0,0];           
    VEH(i).stayt=0;                 % remaining time for vehicle to stay at node
    VEH(i).TimeV=[0,0,0,0,0,0,0,0,0,0,0];   % Time variables (PaxID,request time,Drtt,Exwt0,Extt0,Exwt,Extt,Rwt,Rtt)
%     VEH(i).SlackArch=[0,0,0,0,0];     % archived slack time [t, Chkpnt, initial, available]
    VEH(i).PM=0;                    % performance measure
end

% SlackTemp=VEH(1).SlackT1;   % template for initial slack time setting
CostTemp=VEH(1).Cost1;  % template for initial cost setting

% prepare passenger data structure
cumNumPax=zeros(T,1);   % cumulative number of passenger requests
PAX=struct('id',[],'type',[],'O',[],'D',[],'hDist',[],'vDist',[],'onbrd',[],'drtt',[],'extt',[],'rtt',[],'exwt',[],'rwt',[],'btime',[],'atime',[],'veh',[]);
% IncomingPax=struct;

K=zeros(size(Pax,1),1);   % chosen vehicle for customers

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
        IncomingPax=IdentPax_Fix(cumNumPax(t,1),cumNumPax(t-1,1),Pax,expttc,dwellt,Stops);
        for i=cumNumPax(t-1,1)+1:cumNumPax(t,1)
            PAX(i)=IncomingPax(i-cumNumPax(t-1,1)); % match passenger data structure
        end
    
        % prepare data structure of candidate route per vehicle        
        CAND=struct('Rt',[],'RtCost',[],'perform',[],'Exwt',[],'Extt',[],'WalkT',[],'Pax',[]); % ,'SlackT',[]
        % field explanation of CAND (struct for candidate routes for each operating vehicle)
        % Rt: candidate route information []
        % RtCost: route cost information
        % perform: performance measure of candidate route
        % Exwt: expected wait time of candidate route
        % Extt: expected travel time of candidate route
        % Pax: pax assigned to candidate route
        % SlackT: slack time of candidate route
        a=cumNumPax(t-1,1)+1;   % Starting ID of incoming pax
        b=cumNumPax(t,1);       % Ending ID of incoming px
        
        % conduct insertion heuristic and determine the best vehicle to match
        for j=a:b
            CandRtInfo=zeros(numOpVeh,1);   % prepare array for j's wait time for candidate routes
            for k=1:numOpVeh
                % insertion heuristic
                CAND(k)=Insert_Fix(VEH(k),PAX(j),alpha,beta,gamma,S,walkspeed,rtlength,t);
%                 CandRtInfo(k,1)=k;                   % vehicle ID
%                 CandRtInfo(2,k)=size(CAND(k).Rt,1);  % length of new route (number of stops)
%                 CandRtInfo(3,k)=CAND(k).perform;     % performance measure of new route
                if CAND(k).perform==Inf % route is infeasible if performance measure is undefined
                    CandRtInfo(k,1)=9999; % infeasible
                else
                    CandRtInfo(k,1)=CAND(k).Exwt(CAND(k).Exwt(:,1)==j,2); % feasible
                end
            end
            if sum(CandRtInfo(:,1)<9999)>0 % there exist some routes that can accept new customer
%                 for k=1:numOpVeh
%                     if CandRtInfo(k,2)==0 % veh k is not feasible
% %                         CandRtInfo(5,k)=Inf; % set performance measure increment to infinity
%                         CandRtInfo(k,3)=9999;
%                     else
%                         % calculate increment of performance measure (linear combination of ride time and wait time)
% %                         CandRtInfo(5,k)=CandRtInfo(3,k)-VEH(k).PM; 
% %                         CandRtInfo(k,3)=CAND(k).WalkT(end,4);
%                         CandRtInfo(k,3)=
%                     end
%                 end
                [~,Kj]=min(CandRtInfo(:,1)); % find the route with shortest wait time of j
                K(j,1)=Kj;  % indicate vehicle ID that transports passenger j
                
                % check if wait time of j is longer than headway
                if CandRtInfo(Kj,1)>=vehhdwy
                    t
                end
                
                % update vehicle and route information
                if PAX(j).O(1,2)<PAX(j).D(1,2)
                    VEH(Kj).Rt1=CAND(Kj).Rt;            % route information
                    VEH(Kj).Cost1=CAND(Kj).RtCost(:,1); % segment travel time
                    VEH(Kj).Pax1=CAND(Kj).Pax;          % passenger information
                    VEH(Kj).WalkT1=CAND(Kj).WalkT;      % walking time (including wait and in-vehicle time)
                else
                    VEH(Kj).Rt2=CAND(Kj).Rt;
                    VEH(Kj).Cost2=CAND(Kj).RtCost(:,1);
                    VEH(Kj).Pax2=CAND(Kj).Pax;
                    VEH(Kj).WalkT2=CAND(Kj).WalkT;
                end
                
                % archive time information to TimeV (11 columns)
                VEH(Kj).TimeV(end+1,1:3)=[j,t,PAX(j).drtt];     % Column1-3: PaxID,request time,direct travel time between OD
                VEH(Kj).TimeV(end,10:11)=CAND(Kj).WalkT(1,2:3); % Column10-11: owalkt,dwalkt
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
                
%                 VEH(Kj).Cost=CAND(Kj).RtCost(:,1);
%                 VEH(Kj).ChkCost=CAND(Kj).ChkCost;
                VEH(Kj).PM=CAND(Kj).perform;    % performance measure increment
                VEH(Kj).Exwt=CAND(Kj).Exwt;     % expected wait time
                VEH(Kj).Extt=CAND(Kj).Extt;     % expected in-vehicle time
                VEH(Kj).Rwt=[VEH(Kj).Rwt;j,-CAND(Kj).WalkT(end,2)+dwellt];  % append new passenger to array tracking real wait time
                
                % update passenger information
                PAX(j).veh=Kj;  % vehicle to which passenger is assigned
                PAX(j).exwt=VEH(Kj).Exwt(VEH(Kj).Exwt(:,1)==j,2);   % import exwt
                PAX(j).extt=VEH(Kj).Extt(VEH(Kj).Extt(:,1)==j,2);   % import extt
            else % rejected (no match with any vehicle)
                PAX(j).onbrd=999; 
                numRejPax(t,1)=numRejPax(t,1)+1;
            end
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
        if t>VEH(i).dispatch && VEH(i).stayt>0  % vehicle is operating and staying at a stop
            [VEHI] = Dwell_Fix(VEH(i),tmst,distcnv,t);  % process vehicle information when dwelling
            VEH(i)=VEHI;    % update vehicle information
        elseif t>VEH(i).dispatch && VEH(i).stayt==0 % vehicle is operating and running
            [VEHI,PAX,numServedPax] = VehBrdAlght_Fix(VEH(i),tmst,t,distcnv,PAX,numServedPax,S,Stops,triptime,CostTemp); % process vehicle information when running
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

% process simulation outputs
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

LPK=zeros(1,11);
LPKK=zeros(1,5);
for i=1:numVeh
    LPK=[LPK;VEH(i).TimeV];
    LPKK=[LPKK;VEH(i).WalkT1;VEH(i).WalkT2];
end
LPK=sortrows(LPK,1);
LPK(LPK(:,1)==0,:)=[];
LPKK=sortrows(LPKK,1);
LPKK(LPKK(:,1)==0,:)=[];

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

for i=1:numOpVeh
    NMN(i,1)=max(VEH(i).LoadLog);
end

pl=1;
AvgTDist=0;
for i=1:numOpVeh
    for j=2:size(VEH(i).PaxServed,1)
        AvgTDist(pl,1)=abs(VEH(i).PaxServed(j,5)-VEH(i).PaxServed(j,6))*rtlength/(S-1);
        pl=pl+1;
    end
    for j=2:size(VEH(i).Pax1,1)
        AvgTDist(pl,1)=abs(VEH(i).Pax1(j,5)-VEH(i).Pax1(j,6))*rtlength/(S-1);
        pl=pl+1;
    end
    for j=2:size(VEH(i).Pax2,1)
        AvgTDist(pl,1)=abs(VEH(i).Pax2(j,5)-VEH(i).Pax2(j,6))*rtlength/(S-1);
        pl=pl+1;
    end
end