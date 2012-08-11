% % figure(7)
grid on
% hold on
% 
% ax = 0.3;%1.5;
% % xoffs = ax-0.2;
% % axis ([-1,1,-1,1]*ax+[xoffs,xoffs,0,0])
% axis equal
% 
% 
% TriggerCount = 1;
% start = 1;
% 
% 
% single_doubleOld = 5;

nbsamples = 16;
di = 50;
for i = 1:di:length(com_prw.time) - di
    subplot(1,2,1);
    plot(com.time(i), com.signals.values(i, 1),'k+');    
    hold on;
    plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, nbsamples+1:2*nbsamples));
    subplot(1,2,2);
    hold on;
    plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, 2*nbsamples+1:3*nbsamples));
%     hold on;
%     plot(com_prw.signals.values(i, nbsamples+1:2*nbsamples), com_prw.signals.values(i, 2*nbsamples+1 : 3 * nbsamples));

%     plot(CPr(i,1),CPr(i,2),'m+');
%     plot(CPd(i,1),CPd(i,2),'g+');
%     plot(CPprerun(i,1),CPprerun(i,2),'k+');
%     plot(xr(i,1),xr(i,2),'b.','MarkerSize',5);
%     plot(pd(i,1),pd(i,2),'o','Color','green');
%     plot(pr(i,1),pr(i,2),'r.','MarkerSize',5);
%     plot(virt_foot_point(i,1),virt_foot_point(i,2),'k.','MarkerSize',8);


%     if (single_double(i,1) ~= single_doubleOld)&&(single_double(i,1)==1||single_double(i,1)==2) %max(foottrigger(i:i+di-1)) == 1
%         TriggerCount = TriggerCount + 1;
% %         hold off
% %         plot(CPd(i,1),CPd(i,2),'g+');
% %         %         plot(0,0)
% %         grid on
% %         hold on
% %         axis equal
%         %         axis ([-1,1,-1,1]*ax+[xoffs,xoffs,yoffs,yoffs])
%         %         axis equal
%     end


%     if single_double(i,1) ~= single_doubleOld
%         SPsize = size(suppoly_lines,2);
%         for k = 1:4:SPsize-1
%             if (suppoly_lines(i,k)+suppoly_lines(i,k+1)+suppoly_lines(i,k+2)+suppoly_lines(i,k+3)) ~= 0
%                 JojoLine([suppoly_lines(i,k),suppoly_lines(i,k+1)],[suppoly_lines(i,k+2),suppoly_lines(i,k+3)],'red');
%             end
%         end
%     end


%     title(['Time: ', num2str(time(i))])
%     xoffs = xr(i,1);
%     yoffs = xr(i,2);
%     axis ([-1,1,-1,1]*ax+[xoffs,xoffs,yoffs,yoffs])
%     pause(0.0000001)
%     if start == 1
%         start = 0;
%         pause
%     end

%     single_doubleOld = single_double(i,1);
end

