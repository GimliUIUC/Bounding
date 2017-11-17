function F = drawPicture(t,X,params,book,control)
% --- parameters for eom ---
[L1,l,M1,J1,g,T_sw] = unpackParams(params);

% --- distribute state and time evenly ---
[t,X] = even_sample(t,X,100);
len = length(t);
% joint positions
q0 = X(1,1:3);
q = X(:,1:3);

% --- bookkeeping ---
touchdownList = book.touchdownList;
touchdownTime = book.touchdownTime;

% ---  control parameters ---
stanceTime = control.stanceTime;
flightTime = control.flightTime;
T = 2*(stanceTime + flightTime);

TDcount = 1;
t_touch = touchdownTime(TDcount);         % 1st touchdown time

for i = 1:len
    %% --- plot body and GRF ---
    [t_touch,TDcount] = plot_body_GRF(i,t,X,params,book,control,t_touch,TDcount);
    
    %% --- plot swing leg ---
    % --- state decomposition ---
    q1 = X(i,3);
    s_b = X(i,7);
    s_f = X(i,8);
    
    %     qb1 = X(i,7);   qb2 = X(i,8);   qf1 = X(i,9);   qf2 = X(i,10);
    th = pi/2 + q1;
    p0 = fcn_p0(q(i,:),params);
    p1 = fcn_p1(q(i,:),params);
    
    % --- toe positions ---
    [stanceBool, stanceFoot] = isInStance(t(i),X(i,:),params,book,control);
    if(stanceFoot == -1)        % back stance
%         TDpoint = findTDpoint(t(i),book);
        TDpoint = [touchdownList(TDcount);0];
        p_toe_b = [TDpoint;0];
        p_toe_f =  p1 + rot(th)*gen_p_hip2toe(s_f,control);
    elseif(stanceFoot == 1)     % front stance
%         TDpoint = findTDpoint(t(i),book);
        TDpoint = [touchdownList(TDcount);0];
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
    
    plot(touchdownList,zeros(size(touchdownList)),'rx')
    text(4,-0.2,['t = ',num2str(t(i))]);
    text(4,-0.3,['s_b = ',num2str(s_b)]);
    text(4,-0.4,['s_f = ',num2str(s_f)]);
    text(4,-0.5,['isInStance = ',num2str(stanceBool)]);
%     pause(0.02)
    F(i) = getframe;
    clf
end

end


function TDpoint = findTDpoint(t,book)

temp = abs(t-book.touchdownTime);
ind = find(temp == min(temp));

if(isempty(ind))
    ind = 1;
end

TDpoint = book.touchdownList(ind);

end



