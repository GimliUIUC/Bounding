% ==============================================================
% Simulate hybrid dynamical system
% ==============================================================
function [t,X,book] = runSim(dynamics,hybridEvent,handleEvent, ...
    tmax,params,book,ic,control)

ic_body = ic;

t = [];
X = [];
tLatest = 0;
maxTimeStep = 1e-3;
TDcount = 1;

while (tLatest < tmax)
    tspan = [tLatest, tmax];
    
    % --- Options ---
    options = odeset('Events',...
        @(t,X)hybridEvent(t,X,params,book,control),...
        'MaxStep',maxTimeStep);%,'relTol',1e-7,'Abstol',1e-7);
    
    
    % --- dynamics ---
    [tChart,XChart,tE,XE,iE] = ode45(...
        @(t,X)dynamics(t,X,params,book,control),...
        tspan, ic_body,options);
    
%     tE = tE(end)
%     XE = XE(end,:)
%     iE = iE(end)

    t = [t;tChart];
    X = [X;XChart];
    tLatest = t(end);
    
    if(~isempty(iE))
        if(length(iE)>1)
            eventIdx = 2;
        else
            eventIdx = 1;
        end
        
        % deal with event
        event = iE(eventIdx);
        
        if(event == 1)
            disp('front touchdown');
        elseif(event == 2)
            disp('front liftoff');
        elseif(event == 3)
            disp('back touchdown');
        elseif(event == 4)
            disp('back liftoff');
        else
            disp('unknown event');
        end
        
        [X,book,control] = ...
           handleEvent(event,t,X,params,book, control);


%         F = drawPicture(t,X,params,book,control);
%         a = 0;
    end 
    
    [TDcount] = drawnow1(t,X,params,control,book,event,TDcount)
%     tempplot(t,X);
    a=7;
    
    ic_body = X(end,:);

end
end
