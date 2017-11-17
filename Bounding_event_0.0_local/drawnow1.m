function [TDcount] = drawnow1(t,X,params,control,book,event,TDcount)
% if(length(book.touchdownList)>1)
%     clf(1)
%     clf(2)
% end
% --- state unwrap ---
x = X(end,1);
z = X(end,2);
q1 = X(end,3);
th = pi/2 + q1;
s_b = X(end,7);
s_f = X(end,8);

p0 = fcn_p0(X(end,:),params);
p1 = fcn_p1(X(end,:),params);

%% body
fig1 = figure(1);
plot([p0(1) p1(1)],[p0(2) p1(2)],'color','blue','linewidth',3)
hold on
plot(p0(1),p0(2),'bo'); plot(p1(1),p1(2),'ro');

%% ground
plot([-7 10],[0 0],'k')

hfig = figure(1);
set(hfig,'Position',[0,300,1400,400]);
axis([-1 8 -0.7 1.5])

%% legs
    % --- toe positions ---
    [stanceBool,stanceFoot] = isInStance(t(end),X(end,:),params,book,control);
    if(stanceFoot == -1)        % back stance
        TDpoint = findTDpoint(t(end),book);
        p_toe_b = [TDpoint;0];
        p_toe_f =  p1 + rot(th)*gen_p_hip2toe(s_f,control);
    elseif(stanceFoot == 1)
        TDpoint = findTDpoint(t(end),book);
        p_toe_b =  p0 + rot(th)*gen_p_hip2toe(s_b,control);
        p_toe_f = [TDpoint;0];
    else
        p_toe_b =  p0 + rot(th)*gen_p_hip2toe(s_b,control);
        p_toe_f =  p1 + rot(th)*gen_p_hip2toe(s_f,control);
    end

    % --- plot both front and back legs --- 
    plot([p0(1) p_toe_b(1)],[p0(2) p_toe_b(2)],'b--')
    plot([p1(1) p_toe_f(1)],[p1(2) p_toe_f(2)],'r--')
    plot(p_toe_b(1),p_toe_b(2),'bo')
    plot(p_toe_f(1),p_toe_f(2),'ro')
    
    % --- plot debug info ---
    text(4,-0.2,['t = ',num2str(t(end))]);
    text(4,-0.3,['s_b = ',num2str(s_b)]);
    text(4,-0.4,['s_f = ',num2str(s_f)]);
    text(2,-0.2,['next phase = ', book.phase]);
    
    if(event == 1)
        eventtext = 'front touchdown';
    elseif(event == 2)
        eventtext = 'front liftoff';
    elseif(event == 3)
        eventtext = 'back touchdown';
    elseif(event == 4)
        eventtext = 'back liftoff';
    else
        eventtext = 'unknown event';
    end
    
    text(2,-0.3,['event = ', eventtext]);
    
    fig2 = figure(2);
    subplot(2,2,1)
    plot(t,X(:,1:3))
    legend('x','z','q1')

    subplot(2,2,2)
    plot(t,X(:,4:6))
    legend('dx','dz','dq1')

    subplot(2,2,3)
    plot(t,X(:,7:8))
    legend('s_b','s_f')

    subplot(2,2,4)
    plot(t,X(:,9:10))
    legend('t','T')
    
end

function TDpoint = findTDpoint(t,book)

ind = find(t > book.touchdownTime,1,'last');

if(isempty(ind))
    ind = 1;
end

TDpoint = book.touchdownList(ind);

end


