function [VEH]=Dwell_Fix(VEH,tmst,distcnv,t)
% % INPUTS % %
% VEH: vehicle data structure
% tmst: length of time step
% distcnv: distance conversion factor
% t: current time

% % OUTPUTS % %
% VEH: vehicle data structure

tmseg=VEH.stayt-tmst;   % remainder of staying time after a time step reduced

% input vehicle and passenger information
if VEH.drctn==1
    RtPrsnt=VEH.Rt1;    % current route
    RtCost=VEH.Cost1;   % current segment travel time
    Pax=VEH.Pax1;       % current engaged passenger
else
    RtPrsnt=VEH.Rt2;
    RtCost=VEH.Cost2;
    Pax=VEH.Pax2;
end

% passenger waiting for being picked up at current stop
if RtPrsnt(1,7)~=0 % there should be a change of number of passenger 
    VEH.load=VEH.load+RtPrsnt(1,7); % update vehicle load
    RtPrsnt(1,7)=0;                 % passenger processed
    for j=1:size(Pax,1)
        if Pax(j,5)==RtPrsnt(1,1) && Pax(j,4)==0 % vehicle arrived passenger j's origin and j is waiting for a ride
            VEH.Rtt=[VEH.Rtt;Pax(j,1),0];           % create a space for tracking real-time in-vehicle time
            VEH.TimeV(VEH.TimeV(:,1)==Pax(j,1),8)=VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),2); % archive real wait time of passenger j
            VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),:)=[];   % delete real wait time of passenger j
            Pax(j,4)=1; % change passenger boarding indicator (0->1)
        end
    end
end

% determine whether vehicle stays at or leaves the stop
if tmseg>0 % there is remainder of staying time
    VEH.stayt=tmseg;        % update stayt
    VEH.DistLeft=[0,0];     % vehicle locates at the stop
    
    % update vehicle and route information
    if VEH.drctn==1
        VEH.Rt1=RtPrsnt;    % current route
        VEH.Pax1=Pax;       % current engaged passenger
    else
        VEH.Rt2=RtPrsnt;
        VEH.Pax2=Pax;
    end

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
    
    % passenger waiting for being picked up at current stop
    if RtPrsnt(1,7)~=0 % there should be a change of number of passenger 
        VEH.load=VEH.load+RtPrsnt(1,7); % update vehicle load
        RtPrsnt(1,7)=0;                 % passenger processed
        for j=1:size(Pax,1)
            if Pax(j,5)==RtPrsnt(1,1) && Pax(j,4)==0 % vehicle arrived passenger j's origin and j is waiting for a ride
                VEH.Rtt=[VEH.Rtt;Pax(j,1),0];           % create a space for tracking real-time in-vehicle time
                VEH.TimeV(VEH.TimeV(:,1)==Pax(j,1),8)=VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),2); % archive real wait time of passenger j
                VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),:)=[];   % delete real wait time of passenger j
                Pax(j,4)=1; % change passenger boarding indicator (0->1)
            end
        end
    end
    
    % estimate the future vehicle location after leaving 
    [MidP,dX,dY,~]=Navigate(RtPrsnt(2,2:3),VEH.LocNow,VEH.v,tmseg,distcnv);
    % outputs
    % MidP: future location of vehicle after a time step
    % dX: horizontal distance between E and S
    % dY: vertical distance between E and S
    % treq: required time to travel between E and S with given speed v

    % update vehicle and route information
    VEH.stayt=0;            % staying time is exhausted
    VEH.DistLeft=[dX,dY];   % horizontal and vertical distance to next stop
    VEH.LocNow=MidP;        % current vehicle location
    VEH.RtVstd=[VEH.RtVstd;RtPrsnt(1,:)];   % archive the stop which vehicle left (1)
    VEH.RT=[VEH.RT;RtPrsnt(1,:)];   % archive the stop which vehicle left (2)
    RtPrsnt(1,:)=[];                % delete the stop left
    RtCost(2,1)=RtCost(2,1)+tmseg;  % update the next segment travel time consumed by tmseg 
    RtCost(1,:)=[];                 % delete the segment travel time that vehicle passed through
    if VEH.drctn==1
        VEH.Rt1=RtPrsnt;    % updated route
        VEH.Cost1=RtCost;   % updated segment travel time
        VEH.Pax1=Pax;       % updated engaged passenger information
    else
        VEH.Rt2=RtPrsnt;
        VEH.Cost2=RtCost;
        VEH.Pax2=Pax;
    end
    
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

% update logs
VEH.LocLog(t,:)=VEH.LocNow; % vehicle location log
VEH.LoadLog(t,:)=VEH.load;  % vehicle load log