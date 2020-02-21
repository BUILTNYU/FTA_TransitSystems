function [VEH,PAX,numServedPax]=VehBrdAlght_Fix(VEH,tmst,t,distcnv,PAX,numServedPax,S,Stops,triptime,CostTemp)
% % INPUTS % %
% VEH: vehicle data structure
% tmst: length of time step
% t: current time
% distcnv: distance conversion factor
% PAX: passenger data structure
% numServedPax: number of passnegers served in this time step
% S: number of stops
% Stops: stop information
% triptime: one-way cycle time
% CostTemp: template for segment travel time

% % OUTPUTS % %
% VEH: vehicle data structure
% PAX: passenger data structure
% numServedPax: number of passengers served in this time step

% recall vehicle and passenger information
if VEH.drctn==1
    RtPrsnt=VEH.Rt1;    % current route
    Pax=VEH.Pax1;       % current engaged passenger
else
    RtPrsnt=VEH.Rt2;
    Pax=VEH.Pax2;
end

% estimate the future vehicle location after leaving 
[MidP,dX,dY,treq]=Navigate(RtPrsnt(1,2:3),VEH.LocNow,VEH.v,tmst,distcnv);
% outputs
% MidP: future location of vehicle after a time step
% dX: horizontal distance between E and S
% dY: vertical distance between E and S
% treq: required time to travel between E and S with given speed v

% determine whether vehicle keeps running or arrives at the stop
if treq<=tmst % vehicle will arrive at the next stop in this time step
    % update veh location
    VEH.LocNow=RtPrsnt(1,2:3);
    VEH.DistLeft=[0,0];

    % update real wait and in-vehicle time before processing pax at stop
    VEH.Rwt(:,2)=VEH.Rwt(:,2)+treq; % add required time to wait time tracking in real-time
    VEH.Rtt(:,2)=VEH.Rtt(:,2)+treq; % add required time to in-vehicle time tracking in real-time
    
    % update exwt and extt in real-time
    for i=1:size(VEH.TimeV,1)
        if VEH.TimeV(i,6)>0     % expected wait time remains (passenger is waiting)
            VEH.TimeV(i,6)=VEH.TimeV(i,6)-treq;     % deduct required time from expected wait time
        elseif VEH.TimeV(i,6)<0
            VEH.TimeV(i,6)=0;       % correct error
        end
        if VEH.TimeV(i,6)==0    % no expected wait time
            if VEH.TimeV(i,7)>0 % expected in-vehicle time remains (passenger is onboard)
                VEH.TimeV(i,7)=VEH.TimeV(i,7)-treq; % deduct required time from expected in-vehicle time
            elseif VEH.TimeV(i,7)<0
                VEH.TimeV(i,7)=0;   % correct error
            end
        end
    end

    % identify passengers who board or alight
    for j=1:size(Pax,1)
        if Pax(j,5)==RtPrsnt(1,1) && Pax(j,4)==0 % vehicle arrived passenger j's origin and j boards
            VEH.Rtt=[VEH.Rtt;Pax(j,1),0];       % create a space for tracking real-time in-vehicle time
            VEH.TimeV(VEH.TimeV(:,1)==Pax(j,1),8)=VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),2);    % archive real wait time
            VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),:)=[];   % delete real wait time of passenger j
            Pax(j,4)=1; % change passenger boarding indicator (0->1)
        elseif Pax(j,6)==RtPrsnt(1,1) && Pax(j,4)==1 % vehicle arrived passenger j's destination and j alights
            VEH.TimeV(VEH.TimeV(:,1)==Pax(j,1),9)=VEH.Rtt(VEH.Rtt(:,1)==Pax(j,1),2);    % archive real in-vehicle time
            VEH.Rtt(VEH.Rtt(:,1)==Pax(j,1),:)=[];   % delete real in-vehicle time of passenger j
            Pax(j,4)=2; % change passenger boarding indicator (1->2)
            VEH.PaxServed=[VEH.PaxServed;Pax(j,:)]; % archive passenger served
        end
    end
    
    % delete passengers served
    Pax(Pax(:,4)==2,:)=[];
    Pax=sortrows(Pax,1,'ascend');
    
    % update vehicle information
    VEH.load=VEH.load+RtPrsnt(1,7); % update current vehicle load
    RtPrsnt(1,7)=0; % net change of number of passenger reflected
    if VEH.drctn==1
        VEH.Rt1(1,7)=0;
        VEH.Cost1(1,1)=0;   % remaining segment travel cost is zero
        VEH.Pax1=Pax;       % update engaged passengers
    elseif VEH.drctn==2
        VEH.Rt2(1,7)=0;
        VEH.Cost2(1,1)=0;   % remaining segment travel cost is zero
        VEH.Pax2=Pax;       % update engaged passengers
    end
    VEH.stayt=(tmst-treq)+RtPrsnt(1,4)-t; % stay until departure time
    if VEH.stayt<0  % error check
        [t,VEH.stayt]
    end

    % update information regarding terminal arrival
    if RtPrsnt(1,1)==1 || RtPrsnt(1,1)==S 
        % error check if vehicle load is not zero though it arrived at terminal
        if VEH.load~=0
            t
        end

        % reset current route to initial status
        if VEH.drctn==1     % current vehicle direction: rightward
            VEH.drctn=2;    % divert vehicle direction to leftward
            VEH.Rt1=[Stops,zeros(S,1),3*ones(S,1),20*ones(S,1),zeros(S,1)]; % reset rightward route
            VEH.Rt1(1,4)=VEH.Rt2(end,4);    % set the same departure time for last stop of leftward route and 1st stop of rightward route
            for j=1:S-1
                VEH.Rt1(j+1,4)=triptime/(S-1)*j+VEH.Rt1(1,4);   % set up timetable
            end
            VEH.Cost1=CostTemp; % use template for segment travel time
        elseif VEH.drctn==2     % current vehicle direction: leftward
            VEH.drctn=1;        % divert vehicle direction to rightward
            VEH.Rt2=[flipud(Stops),zeros(S,1),3*ones(S,1),20*ones(S,1),zeros(S,1)]; % reset leftward route
            VEH.Rt2(1,4)=VEH.Rt1(end,4);    % set the same departure time for last stop of leftward route and 1st stop of rightward route
            for j=1:S-1
                VEH.Rt2(j+1,4)=triptime/(S-1)*j+VEH.Rt2(1,4);   % set up timetable
            end
            VEH.Cost2=CostTemp; % use template for segment travel time
        end
    end

    % update Rtt, Rwt after processing
    VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst-treq;    % add remaining time of time step to wait time tracking in real-time
    VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst-treq;    % add remaining time of time step to in-vehicle time tracking in real-time

    % update exwt and extt in real-time
    for i=1:size(VEH.TimeV,1)
        if VEH.TimeV(i,6)>0
            VEH.TimeV(i,6)=VEH.TimeV(i,6)-(tmst-treq);
        else
            if VEH.TimeV(i,7)>0
                VEH.TimeV(i,7)=VEH.TimeV(i,7)-(tmst-treq);
            end
        end
    end
else % vehicle will keep moving
    VEH.LocNow=MidP;         % updated vehicle location
    VEH.DistLeft=[dX,dY];    % distance to next stop
    
    % update Cost, Rtt, Rwt
    if VEH.drctn==1
        VEH.Cost1(1,1)=VEH.Cost1(1,1)-tmst;
    elseif VEH.drctn==2
        VEH.Cost2(1,1)=VEH.Cost2(1,1)-tmst;
    end
    VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst;
    VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst;
    
    % update exwt and extt in real-time
    for i=1:size(VEH.TimeV,1)
        if VEH.TimeV(i,6)>0
            VEH.TimeV(i,6)=VEH.TimeV(i,6)-tmst;
        else
            if VEH.TimeV(i,7)>0
                VEH.TimeV(i,7)=VEH.TimeV(i,7)-tmst;
            end
        end
    end
end

% update logs
VEH.LocLog(t,:)=VEH.LocNow; % vehicle location log
VEH.LoadLog(t,:)=VEH.load;  % vehicle load log