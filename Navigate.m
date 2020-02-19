function [MidP,dX,dY,treq]=Navigate(E,S,v,tmseg,distcnv) % vehicle movement: vertical movment first, horizontal movement next
% % INPUTS % %
% E: End (destination)
% S: Start (origin)
% v: Speed (mph)
% tmseg: Length of time segment for vehicle moving (sec)
% distcnv: Conversion factor of distance (mi/coord unit)

% % OUTPUTS % %
% MidP: future location of vehicle after a time step
% dX: horizontal distance between E and S
% dY: vertical distance between E and S
% treq: required time to travel between E and S with given speed v

dY=E(1,2)-S(1,2);       % vertical distance
dYt=abs(dY)/(v/3600);   % time to travel dY
dX=E(1,1)-S(1,1);       % horizontal distance
dXt=abs(dX)/(v/3600);   % time to travel dX
treq=dYt+dXt;           % time to travel dY and dX

if treq>tmseg           % remained distance cannot be covered within given time
    if dYt>tmseg        % vertical distance cannot be covered within given time
        MidP(1,1)=S(1,1);   % x coord remains the same as that of starting point
        MidP(1,2)=S(1,2)+sign(dY)*v/3600*tmseg/distcnv; % y coord changes
    else                % vertical distance can be covered within given time
        MidP(1,2)=E(1,2);   % y coord remains the same as that of end point
        if dYt==0
            MidP(1,1)=S(1,1)+sign(dX)*v/3600*tmseg/distcnv; % x coord changes
        elseif dYt>0
            MidP(1,1)=S(1,1)+sign(dX)*v/3600*(tmseg-dYt)/distcnv; % x coord changes
        end
    end
elseif treq<=tmseg     % remained distance can be covered within given time
    MidP=E; % final position will be end point
end