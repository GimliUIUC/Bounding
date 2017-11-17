% toy example
% function test_aerial_phase()
L1 = 0.22;
L2 = 0.22;
len = 100;

coeff_x = [-0.1 -0.16 -0.1  -0.07  -0.02 0.02  0.07   0.1  0.16 0.1];
coeff_z = [-0.35 -0.25 -0.18  -0.2 -0.32 -0.32 -0.2 -0.18 -0.25 -0.35];


s = linspace(0,1,len);
pos_x = polyval_bz(coeff_x, s);
pos_z = polyval_bz(coeff_z, s);
dx = polyval_bz_d(coeff_x, s)';
dz = polyval_bz_d(coeff_z, s);
p = [pos_x;pos_z];
diff(pos_x)';

hfig = figure;
set(hfig,'Position',[300,300,400,400]);
plot(pos_x,pos_z)
axis([-0.3 0.3 -0.5 0.1])


% q = zeros(0,2);
% for ind = 1:len
%     q(end+1,:) = invKin(p(:,ind),L1,L2);
% end
% 
% %% --- plot ---
% hold on
% 
% 
% for j = 1:len
% %     plot(pos_x(j),pos_z(j),'o')
%     hold on
%     q1 = q(j,1);    q2 = q(j,2);
%     P1 = [-L1*cos(q1),-L1*sin(q1)];
%     P2 = [-L1*cos(q1)-L2*cos(q1+q2),-L1*sin(q1)-L2*sin(q1+q2)];
%     plot([0 P1(1) P2(1)],[0 P1(2) P2(2)])
%     axis([-0.3 0.3 -0.6 0])
%     plot(pos_x,pos_z)
%     xlabel('x [m]')
%     ylabel('z [m]')
%     title('Toe Trajectory Relative to Shoulder in Swing Phase')
%     grid on
%     box on
%     
%     pause(0.01);
%     F(j) = getframe;
%     clf
% end

% -- videos ---
% v = VideoWriter('new_video.avi');
% open(v)
% writeVideo(v,F)
% close(v)



