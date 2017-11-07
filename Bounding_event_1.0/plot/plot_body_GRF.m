function [t_touch,TDcount] = ...
    plot_body_GRF(i,t,X,params,book,control,t_touch,TDcount)

[L1,l,M1,J1,g] = unpackParams(params);

% ---  control parameters ---
stanceTime = control.stanceTime;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
coeff1 = control.coeff1;
coeff2 = control.coeff2;

% --- bookkeeping ---
touchdownList = book.touchdownList;

% --- state decomposition ---
x = X(i,1);
z = X(i,2);
q1 = X(i,3);

% --- joint positions ---
p_COM = [x;z];
p0 = p_COM - rot(q1) * [0;L1/2];
p1 = p_COM + rot(q1) * [0;L1/2];

% --- plot body and legs ---
% body
hfig = figure(2);
set(hfig,'Position',[0,300,1400,400]);

plot([p0(1) p1(1)],[p0(2) p1(2)],'color','blue','linewidth',3)
hold on
plot(p0(1),p0(2),'bo'); plot(p1(1),p1(2),'ro');
plot(p_COM(1),p_COM(2),'go');
% ground
plot([-7 10],[0 0],'k')
% --- constrain the view ---
axis([-1 8 -0.7 1.5])

%% --- plot GRF ---
if((t_touch <= t(i)) && (t(i) <= (t_touch + stanceTime)))
    % percentage completion
    s = (t(i) - t_touch)/stanceTime;
    b = bezier(s,coeff1,coeff2);
    
    if(mod(TDcount,2) ~= 0)       % hind foot
        tau = alpha_tau * b;
        color = 'blue';
    else                   % front foot
        tau = - alpha_tau * b;
        color = 'red';
    end
    % point of landing
    landpoint = [touchdownList(TDcount);0];
    r = landpoint - [x;z]; % vector from COM to toe
    
    % --- GRF ---
    Fz = alpha_z * b; 
    Fx = (r(1)*Fz - tau)/(r(2));
    Fe = [Fx;Fz]/400;
    plot([landpoint(1) landpoint(1)+Fe(1)],[0 Fe(2)],'Color',color);
    
elseif(t_touch + stanceTime < t(i))
    % move to the next touchdown time
    TDcount = TDcount + 1;
    if(~isempty(book.touchdownTime(TDcount)))
        t_touch = book.touchdownTime(TDcount);
    end
    
end

end