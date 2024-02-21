clc; close; clear;

%% Değerler
% Başlangıç pose [x0,y0] ve hedef pose [xg,yg]
x0 = 0;
y0 = 0;
xg = 10;
yg = 20;

% Robot parametreleri
R = 0.1;
L = 0.3;

% Kontrol parametreleri
Kp = 1.0;
Kh = 1.0;

% Zaman değerleri
dt = 0.1;
t = 0;

% Başlangıç değerleri
x = x0;
y = y0;
theta = 0;  % radyan
v = 0;      % m/s

% Listeler
x_data = [];
y_data = [];
th_data = [];
v_data = [];
w_data = [];

%% Koordinatlar
while true
    delta_x = xg - x;
    delta_y = yg - y;
    theta_g = atan2(delta_y, delta_x);

    delta_theta = theta_g - theta;
    delta_theta = atan2(sin(delta_theta), cos(delta_theta)); % Açıyı -pi ile pi arasında tut
    uzaklik = sqrt(delta_x^2 + delta_y^2);

    % Hız ve dönüş açısı hesaplama
    v = Kp * uzaklik;
    omega = Kh * delta_theta;
    
    % Hızı sınırla
    if v > 1
        v = 1;
    end
    if omega > pi/4
        omega = pi/4;
    end

    % Yeni pozisyon ve açıyı hesapla
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    t = t + dt;

    % Sonuçları göster
    fprintf('t=%.1f, x=%.3f, y=%.3f, theta=%.4f, v=%.3f, omega=%.3f\n', t, x, y, theta*(180/pi), v, omega*(180/pi));
    
    % Sonuçları listeye kaydet
    x_data = [x_data, x];
    y_data = [y_data, y];
    th_data = [th_data, theta];
    v_data = [v_data, v];
    w_data = [w_data, omega];

    % Döngüyü sonlandır
    if abs(delta_x) < 0.0001 && abs(delta_y) < 0.0001
        break;
    end
end

%% Çizim ve Simülasyon

plot(x_data, y_data, 'k--');
title('Rota ve Simülasyon');
xlabel('x (m)');
ylabel('y (m)');
hold on;
wls=[0 0;-L L];
wlsrot = [0, 0; 0, 0]*wls;
h1=plot(wlsrot(1,1),wlsrot(2,1),'ro','LineWidth',2,'MarkerFaceColor','r');
h2=plot(wlsrot(1,2),wlsrot(2,2),'ro','LineWidth',2,'MarkerFaceColor','r');
h3=plot(x_data(1),y_data(1),'bo','MarkerSize',20);
plot(xg,yg,'go','MarkerSize',5,'MarkerFaceColor','g');
plot(x0,y0,'go','MarkerSize',5,'MarkerFaceColor','g');

% Ok ekleme
arrow_length = 0.3;
arrow_head_length = 0.5;
quiver_scale = 4;

arrow_x = arrow_length * cos(th_data(1));
arrow_y = arrow_length * sin(th_data(1));
quiver_x = x_data(1) + arrow_x;
quiver_y = y_data(1) + arrow_y;
quiver_dx = quiver_scale * arrow_x;
quiver_dy = quiver_scale * arrow_y;

quiver_arrow = quiver(quiver_x, quiver_y, quiver_dx, quiver_dy, 0, 'g', 'MaxHeadSize', arrow_head_length);

axis([-1.25 1.25 -0.5 2]);
xlim([0 11]);
ylim([0 21]);
t_length = length(0:dt:t);
for i = 2:t_length
    wlsrot = [cos(th_data(i)), -sin(th_data(i)); sin(th_data(i)), cos(th_data(i))] * wls;
    set(h1, 'XData', wlsrot(1, 1) + x_data(i));
    set(h1, 'YData', wlsrot(2, 1) + y_data(i));
    set(h2, 'XData', wlsrot(1, 2) + x_data(i));
    set(h2, 'YData', wlsrot(2, 2) + y_data(i));
    set(h3, 'XData', x_data(i));
    set(h3, 'YData', y_data(i));

    % Ok pozisyonunu güncelleme
    arrow_x = arrow_length * cos(th_data(i));
    arrow_y = arrow_length * sin(th_data(i));
    quiver_x = x_data(i) + arrow_x;
    quiver_y = y_data(i) + arrow_y;
    set(quiver_arrow, 'XData', quiver_x, 'YData', quiver_y, 'UData', arrow_x * quiver_scale, 'VData', arrow_y * quiver_scale);
    
    drawnow;
    pause(0.03);
end
