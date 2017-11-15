function [X,book,control] = ...
    handleEvent(event,t,X,params,book,control) 

tE = t(end);
XE = X(end,:);
[x, z, q1, dx, dz, dq1, s_b, s_f, t, T] = unpackState(XE);

th = pi/2 + q1;
p0 = fcn_p0(XE,params);
p1 = fcn_p1(XE,params);
toe_b = p0 + rot(th) * gen_p_hip2toe(s_b,control);
toe_f = p1 + rot(th) * gen_p_hip2toe(s_f,control);

if(event == 1)  % front touchdown
    book.touchdown_front = tE;
    book.touchdownTime(end+1) = tE;
    book.touchdownPoint = toe_f(1);
    book.touchdownList(end+1) = book.touchdownPoint;
    book.phase = 'frontStance';
    XE = fcn_h3_event(X(end,:),params);
    control.stanceTime = XE(10);
    
elseif(event == 2)  % front liftoff
    book.liftoff_front = tE;
    book.liftoffTime(end+1) = tE;
    control.foot = -1;
    book.phase = 'Aerial_1';
    XE = fcn_h4_event(X(end,:),params);

elseif(event == 3)  % back touchdown
    book.touchdown_back = tE;
    book.touchdownTime(end+1) = tE;
    book.touchdownPoint = toe_b(1);
    book.touchdownList(end+1) = book.touchdownPoint;
    book.phase = 'backStance';
    XE = fcn_h1_event(X(end,:),params);
    control.stanceTime = XE(10);

elseif(event == 4)  % back liftoff
    book.liftoff_back = tE;
    book.liftoffTime(end+1) = tE;
    control.foot = 1;
    book.phase = 'Aerial_2';
    XE = fcn_h2_event(X(end,:),params);
end
    X(end,:) = XE;


end