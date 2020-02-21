function [VEH]=Dwell_DtD(VEH,tmst,distcnv,t)
% % INPUTS % %
% VEH: vehicle data structure
% tmst: length of time step
% distcnv: distance conversion factor
% t: current time

% % OUTPUTS % %
% VEH: vehicle data structure


tmseg=VEH.stayt-tmst;   % remainder of staying time after a time step reduced

% recall vehicle and passenger information
RtPrsnt=VEH.Rt1;        % current route
RtCost=VEH.Cost1;       % current segment travel time
RtCumCost=VEH.CumCost1; % current cumulative segment travel time
Pax=VEH.Pax1;           % current engaged passenger

% determine whether vehicle stays at or leaves the stop
if tmseg>0 % there is remainder of staying time
    VEH.stayt=tmseg;    % update stayt
    if VEH.idle==1  % vehicle has no stop to go
        VEH.stayt=9999; % assign big number of staying time so vehicle can stay at the stop as long as it can
    end
    VEH.DistLeft=[0,0];         % vehicle locates at the stop
    RtCumCost=RtCumCost-tmst;   % deduct a time step from cumulative segment travel time
    VEH.CumCost1=RtCumCost;     % update cumulative segment travel time

    % update real wait and in-vehicle time
    VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst; % add time step to wait time tracking in real-time
    VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst; % add time step to in-vehicle time tracking in real-time
    
    % update exwt and extt in real-time
    for i=1:size(VEH.TimeV,1)
        if VEH.TimeV(i,6)>0     % deduct only expected wait time if remaining
            VEH.TimeV(i,6)=VEH.TimeV(i,6)-tmst;     % deduct a time step
        else
            if VEH.TimeV(i,7)>0 % deduct remaining expected in-vehicle time
                VEH.TimeV(i,7)=VEH.TimeV(i,7)-tmst; % deduct a time step
            end
        end
    end

else % staying time is exhausted wihtin this time step and vehicle leaves
    % error check if vehicle load is negative
    if VEH.load<0
        t
    end
    
    % determine next stop to visit
    if size(RtPrsnt,1)>=2   % vehicle has at least one stop to visit next
        [MidP,dX,dY,~]=Navigate(RtPrsnt(2,2:3),VEH.LocNow,VEH.v,tmseg,distcnv); % estimate the future vehicle location after leaving 

        % update information
        VEH.stayt=0;            % staying time is exhausted
        VEH.DistLeft=[dX,dY];   % horizontal and vertical distance to next stop
        VEH.LocNow=MidP;        % current vehicle location
        VEH.RtVstd=[VEH.RtVstd;RtPrsnt(1,:)];   % archive the stop which vehicle left (1)
        VEH.RT=[VEH.RT;RtPrsnt(1,:)];   % archive the stop which vehicle left (2)
        RtPrsnt(1,:)=[];                % delete the stop left
        VEH.Rt1=RtPrsnt;        % update route
        VEH.Pax1=Pax;           % update engaged passenger information

        RtCost(2,1)=RtCost(2,1)+tmseg;  % update the next segment travel time consumed by tmseg
        RtCumCost=RtCumCost-tmst;       % update cumulative segment travel time consumed by tmseg
        RtCost(1,:)=[];                 % delete the segment travel time that vehicle passed through
        RtCumCost(1,:)=[];              % delete cumulative travel time that vehicle passed through
        VEH.Cost1=RtCost;               % update segment travel time
        VEH.CumCost1=RtCumCost;         % update cumulative segment travel time
        % update real wait and in-vehicle time
        VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst; % add time step to wait time tracking in real-time
        VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst; % add time step to in-vehicle time tracking in real-time
    
        % update exwt and extt in real-time
        for i=1:size(VEH.TimeV,1)
            if VEH.TimeV(i,6)>0     % deduct only expected wait time if remaining
                VEH.TimeV(i,6)=VEH.TimeV(i,6)-tmst;     % deduct a time step
            else
                if VEH.TimeV(i,7)>0 % deduct remaining expected in-vehicle time
                    VEH.TimeV(i,7)=VEH.TimeV(i,7)-tmst; % deduct a time step
                end
            end
        end
    
    else % vehicle returns to depot because there is no stop to visit
        [MidP,dX,dY,~]=Navigate(VEH.Depot,VEH.LocNow,VEH.v,tmseg,distcnv);  % estimate the future vehicle location after leaving 
        
        % update information
        VEH.idle=1;             % vehicle has no route to operate
        VEH.stayt=0;            % staying time is exhausted
        VEH.DistLeft=[dX,dY];   % horizontal and vertical distance to next stop
        VEH.LocNow=MidP;        % current vehicle location
        VEH.RtVstd=[VEH.RtVstd;RtPrsnt(1,:)];   % archive the stop which vehicle left (1)
        VEH.RT=[VEH.RT;RtPrsnt(1,:)];   % archive the stop which vehicle left (2)
        RtPrsnt(1,:)=[];                % delete the stop left
        VEH.Rt1=RtPrsnt;        % update route
        VEH.Pax1=Pax;           % update engaged passenger information

%         RtCost(2,1)=RtCost(2,1)+tmseg;
%         RtCumCost=RtCumCost-tmst;
        RtCost(1,:)=[];                 % delete the segment travel time that vehicle passed through
        RtCumCost(1,:)=[];              % delete cumulative travel time that vehicle passed through
        VEH.Cost1=RtCost;               % update segment travel time
        VEH.CumCost1=RtCumCost;         % update cumulative segment travel time
        % update real wait and in-vehicle time
        VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst; % add time step to wait time tracking in real-time
        VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst; % add time step to in-vehicle time tracking in real-time
        % update exwt and extt in real-time
        for i=1:size(VEH.TimeV,1)
            if VEH.TimeV(i,6)>0     % deduct only expected wait time if remaining
                VEH.TimeV(i,6)=VEH.TimeV(i,6)-tmst;     % deduct a time step
            else
                if VEH.TimeV(i,7)>0 % deduct remaining expected in-vehicle time
                    VEH.TimeV(i,7)=VEH.TimeV(i,7)-tmst; % deduct a time step
                end
            end
        end
    end
end

% update logs
VEH.LocLog(t,:)=VEH.LocNow; % vehicle location log
VEH.LoadLog(t,:)=VEH.load;  % vehicle load log