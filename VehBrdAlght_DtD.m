function [VEH,PAX,numServedPax]=VehBrdAlght_DtD(VEH,tmst,distcnv,PAX,numServedPax,t)
% % INPUTS % %
% VEH: vehicle data structure
% tmst: length of time step
% distcnv: distance conversion factor
% PAX: passenger data structure
% numServedPax: number of passnegers served in this time step
% t: current time

% % OUTPUTS % %
% VEH: vehicle data structure
% PAX: passenger data structure
% numServedPax: number of passengers served in this time step


% recall vehicle and passenger information
RtPrsnt=VEH.Rt1;        % current route
% RtCost=VEH.Cost1;       % current segment travel time
CumCost=VEH.CumCost1;   % current cumulative segment travel time
Pax=VEH.Pax1;           % current engaged passenger

% determine next stop to visit
if size(RtPrsnt,1)>0    % vehicle has at least one stop to visit next
    [MidP,dX,dY,treq]=Navigate(RtPrsnt(1,2:3),VEH.LocNow,VEH.v,tmst,distcnv);   % estimate the future vehicle location 
    % determine arrival
    if treq<=tmst % vehicle will arrive at the next stop in this time step
        % update veh location
        VEH.LocNow=RtPrsnt(1,2:3);  % vehicle location is the same as next stop
        VEH.DistLeft=[0,0];         % vehicle arrives at next stop
        CumCost=CumCost-tmst;       % deduct a time step from cumulative segment travel time
        
        % update Rwt, Rtt before processing pax at stop
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
            if Pax(j,2)==RtPrsnt(1,1) && Pax(j,4)==0 % vehicle arrived passenger j's origin and j boards
                VEH.Rtt=[VEH.Rtt;Pax(j,1),0];       % create a space for tracking real-time in-vehicle time
                VEH.TimeV(VEH.TimeV(:,1)==Pax(j,1),8)=VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),2);    % archive real wait time
                VEH.Rwt(VEH.Rwt(:,1)==Pax(j,1),:)=[];   % delete real wait time of passenger j
                Pax(j,4)=1; % change passenger boarding indicator (0->1)
            elseif Pax(j,3)==RtPrsnt(1,1) && Pax(j,4)==1 % vehicle arrived passenger j's destination and j alights
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
%         RtPrsnt(1,7)=0;         % net change of number of passenger reflected
        VEH.Rt1(1,7)=0;         % net change of number of passenger reflected
        VEH.Pax1=Pax;           % update engaged passengers
        VEH.Cost1(1,1)=0;       % segment travel time is zero
        VEH.CumCost1=CumCost;   % update cumulative segment travel time

        VEH.stayt=VEH.Rt1(1,6)-(tmst-treq); % staying time is reduced dwellt at stop
%         if size(VEH.Rt1,1)==1
%             VEH.stayt=VEH.Rt1(1,6);
%         else
%             VEH.stayt=dwellt-(tmst-treq); % dwell
%         end
        
        if VEH.stayt<0  % error check
            [t,VEH.stayt]
        end

        % update Rtt, Rwt after processing
        VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst-treq;    % add remaining time step to wait time tracking in real-time
        VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst-treq;    % add remaining time step to in-vehicle time tracking in real-time
        
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

        % vehicle information should be updated

    else % vehicle will keep moving
        VEH.LocNow=MidP;         % updated veh location
        VEH.DistLeft=[dX,dY];    % distance to next stop

        % update Cost, CumCost, Rtt, Rwt
        VEH.Cost1(1,1)=VEH.Cost1(1,1)-tmst; % deduct time step
        VEH.CumCost1=VEH.CumCost1-tmst;     % deduct time step
        VEH.Rtt(:,2)=VEH.Rtt(:,2)+tmst;     % add time step
        VEH.Rwt(:,2)=VEH.Rwt(:,2)+tmst;     % add time step
        
        
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
else % vehicle returns to depot because there is no stop to visit
    [MidP,dX,dY,treq]=Navigate(VEH.Depot,VEH.LocNow,VEH.v,tmst,distcnv);    % estimate the future vehicle location
    % determine arrival
    if treq<=tmst % vehicle will arrive at the depot in this time step
        VEH.LocNow=VEH.Depot;   % vehicle location is the same as depot
        VEH.DistLeft=[0,0];     % vehicle arrives at next stop
        VEH.stayt=9999;         % assign big number of staying time so vehicle can stay at the stop as long as it can
%         CumCost=CumCost-tmst;
    else % vehicle is heading to the depot
        VEH.LocNow=MidP;        % update vehicle location
        VEH.DistLeft=[dX,dY];   % update distance left
    end
end

% update logs
VEH.LocLog(t,:)=VEH.LocNow; % vehicle location log
VEH.LoadLog(t,:)=VEH.load;  % vehicle load log