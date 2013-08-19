format long
a=dlmread('sensor0.txt');
b=dlmread('sensor1.txt');


h=figure;
plot(max(a(:,1))-a(:,1),a(:,2),'k');
hold on;
plot(max(b(:,1))-b(:,1),b(:,2),'b');
title('IR response in maze (front sensors)');
xlabel('Distance from wall');
ylabel('Sensor read [a.u.]');
xlim([0 0.56])
legend({'Front right', 'Front left'}, ...
    'Location', 'NorthEast');
saveas(h,'IR_front.png');

h2=figure;
plot(max(a(:,1))-a(:,1),a(:,2),'k');
hold on;
plot(max(b(:,1))-b(:,1),b(:,2),'b');
title('IR response in maze (front sensors)');
xlabel('Distance from wall');
ylabel('Sensor read [a.u.]');
xlim([0 0.04])
legend({'Front right', 'Front left'}, ...
    'Location', 'NorthEast');
saveas(h2,'IR_front_lm.png');


% figure
% plot(max(c(:,1))-c(:,1),c(:,2),'k+');
% hold on;
% plot(max(d(:,1))-d(:,1),d(:,2),'b+');
% title('IR response in maze (45^{\circ} sensors)');
% xlabel('Distance from wall');
% ylabel('Sensor read [a.u.]');
% legend({'Right', 'Left'}, ...
%     'Location', 'NorthEast');
