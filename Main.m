% "Main.m" is a control panel of the simulation. User can adjust simulation
% parameters and choose a transit system to simulate among three options;
% fixed route, flexible route, and door-to-door service.
% Please locate all MATLAB codes and dataset in the same folder.

% 1. Select demand level and system
demand=400; % demand level (80 pax/h / 200 pax/h / 400 pax/h)
system=2;   % system indiator (1: fixed route/2: flexible route/3: door-to-door service)

% 2. Simulation parameters 
% 2.1. Give route length first
rtlength=8.2;	% length of route (L) (one-way)

% 2.2. Define system-specific parameters
if system==1 % fixed route
    triptime=5280;	% one-way cycle time: 5280 sec (88 min) for fixed route
    numVeh=15;      % required fleet size (V) for maintaining frequency of 5 veh/h
    vehcap=85;      % vehicle capacity (general MTA buses)
    numOpVeh=1;     % number of currently operating vehicles
    nStops=57;      % number of all stops including terminals
    Stops=[(1:nStops)',((1:nStops)'-1)*(rtlength/(nStops-1)),zeros(nStops,1)];   % stop information (id,x,y)
elseif system==2 % flexible route
    triptime=7200;	% one-way cycle time: 7200 sec (120 min) for flexible route
    numVeh=20;      % required fleet size for maintaining frequency of 5 veh/h
    numOpVeh=1;     % number of currently operating vehicles
    vehcap=40;      % vehicle capacity (smaller than MTA buses)
    nChks=8;        % number of intermediate checkpoints
    C=nChks+2;      % number of all checkpoints including terminals
    ChkPnts=[(1:C)',((1:C)'-1)*(rtlength/(C-1)),zeros(C,1)];   % checkpoint information (id,x,y)
    maxback=0.25;   % maximum backtracking distacne per checkpoint segment
    maxwait=720;    % maximum wait time
elseif system==3 % door-to-door service
    triptime=0;     % there is no cycle for door-to-door service
    numVeh=40;      % double fleet size than flexible route instead of reducing capacity per vehicle
    numOpVeh=numVeh;     % number of currently operating vehicles
    vehcap=20;      % vehicle capacity (mini buses)
    nDpts=18;        % number of intermediate checkpoints
    D=nDpts+2;      % number of all checkpoints including terminals
    Depots=[(1:D)',((1:D)'-1)*(rtlength/(D-1)),zeros(D,1)];   % checkpoint information (id,x,y)
    maxdelay=2;     % maximum detour delay (zeta_t)(ratio compared to direct travel time)
    maxwait=1800;   % maximum wait time (zeta_w)(sec)
end

% 2.3. Define common parameters 
rtlength=8.2;	% length of route (L) (one-way)
tmst=1;         % time step (tau): 1 sec
ophr=4;         % system operation hour (hr)
dwellt=20;      % average dwell time at stops (t_d) (sec)
warmupt=triptime*2;     % time for warming up (sec) (locate vehicles along the route)
T=3600*ophr+warmupt;    % simulation time (sec)
vehhdwy=720;    % vehicle headway (sec) from frequency of 5 veh/h
vehVmph=7.13;   % average vehicle speed (v_o)(mi/h)=(mi/3600sec)
vehVmps=vehVmph/3600;  % average vehicle speed (mi/sec)
% shortesttime=rtlength/vehVmps; % shortest one-way trip time (sec)
% FixedStops=[1,0,0;(2:51)',0.2*(1:50)',zeros(50,1)]; % coordination of fixed stops
avgReq=demand/3600; % average number of passenger per sec

nStops=57;      % number of stops on existing fixed route
expttc=1/vehVmps;      % expected travel time cost (pace, sec/mi)
walkspeed=3.1;      % walking speed (mph)
walklimit=0.5;      % maximum distance for walking (mi)

% service area: rectangle
VRange=[-walklimit,walklimit];  % vertical range of region (mi), 
HRange=[0,rtlength];            % horizontal range of region (mi)
distcnv=1;                      % distance conversion factor: 1 unit in coordination = 1 mi

% weight for travel time (Wardman, 2004)
alpha=1;    % weight for in-vehicle time (gamma_v)
beta=1.59;   % weight for wait time (gamma_w)
gamma=1.79;  % weight for walking time (gamma_a)

% 3. passenger information preparation
% 3.1. If passenger information is prepared in advance, import it
load(sprintf('Pool_%d_Random.mat',demand));   % load demand dataset
% 3.2. If generating a new passenger set, use this function
% [Pax,numNewPax]=PaxGen(HRange,VRange,walklimit,avgReq,nStops); % randomized bidirectional

% 3.3. plot pax locations to visually verify generated pax pool (not necessary)
% Pplot=zeros(1,5);
% for i=1:size(Pax,1)
%     Pplot(i,1)=i;
%     Pplot(i,2:3)=PAX(i).O(1,3:4);
%     Pplot(i,4:5)=PAX(i).D(1,6:7);
% end
% hold on
% for i=1:size(Pax,1)
%     plot([Pplot(i,2),Pplot(i,4)],[Pplot(i,3),Pplot(i,5)],'color',[0.2,0.2,0.2])
% end
% scatter(Pplot(:,2),Pplot(:,3),20,'r')
% scatter(Pplot(:,4),Pplot(:,5),20,'b')
% PplotID=Pplot(:,1);
% c = num2str(PplotID);
% dx = 0.1; dy = 0.02; % displacement so the text does not overlay the data points
% text(Pplot(:,2)+dx, Pplot(:,3)+dy, c);
% text(Pplot(:,4)+dx, Pplot(:,5)+dy, c);
% hold off

% 4. Run simulation for chosen system
if system==1 % fixed route
    [VEH,PAX]=FixedRoute(Pax,numNewPax,rtlength,tmst,triptime,distcnv,dwellt,warmupt,T,numOpVeh,numVeh,vehcap,vehhdwy,vehVmph,vehVmps,Stops,expttc,walkspeed,walklimit,VRange,HRange,alpha,beta,gamma);
elseif system==2 % flexible route
	[VEH,PAX]=FlexibleRoute(Pax,numNewPax,rtlength,tmst,triptime,distcnv,dwellt,warmupt,T,numOpVeh,numVeh,vehcap,vehhdwy,vehVmph,vehVmps,ChkPnts,expttc,walkspeed,walklimit,VRange,alpha,beta,maxback,maxwait);
elseif system==3 % door-to-door service
    [VEH,PAX]=DtD(Pax,numNewPax,rtlength,tmst,distcnv,dwellt,T,numOpVeh,numVeh,vehcap,vehVmph,expttc,alpha,beta,maxdelay,maxwait,D,Depots);
end