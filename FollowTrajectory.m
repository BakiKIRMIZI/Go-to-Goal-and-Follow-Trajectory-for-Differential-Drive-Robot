clc; close; clear;

%% Değerler
% Başlangıç pose [x0,y0,th0]
x0 = 0;
y0 = 0;
theta0 = 0;

% Robot parametreleri
R = 0.1;
L = 0.3;

% Kontrol parametreleri
Kv = 1.0;
Ki = 1.0;
Kh = 1.0;

% Zaman değerleri
dt = 0.1;
t = 0;
t_max = 25;

% Başlangıç değerleri
x = x0;
y = y0;
theta = theta0; % radyan
v = 0;          % m/s
w = 0;          % radyan/s
x_g = 0;
y_g = 0;
theta_g = 0;
d = 1;
x_hedef = 0;
y_hedef = 0;
theta_hedef = 0;

% Listeler
x_data = [];
y_data = [];
th_data = [];
v_data = [];
w_data = [];

%% Döngü
while true
    t = t + dt;

    x_g = t;
    y_g = cos(t);
    
    % Hız ve dönüş açısı hesaplama
    e = sqrt((x_g - x)^2 + (y_g - y)^2) - d;
    v = Kv * e + Ki * (e*t - e*(t-dt)); %integral(e,t-dt,t);
    theta_g = atan2(y_g - y, x_g - x);
    w = Kh * (atan2(sin(theta_g - theta), cos(theta_g - theta)));
    
    % Yeni pozisyon ve açıyı hesapla
    x_hedef = x_hedef + v * cos(theta) * dt;
    y_hedef = y_hedef + v * sin(theta) * dt;
    theta_hedef = theta_hedef + w * dt;
    
    % Sonuçları göster
    fprintf('t=%.1f, x=%.3f, y=%.3f, theta=%.4f, v=%.3f, omega=%.3f\n', t, x_hedef, y_hedef, theta_hedef*(180/pi), v, w*(180/pi));

    % Koordinatları güncelle
    x = x_hedef;
    y = y_hedef;
    theta = theta_hedef;


    % Sonuçları listeye kaydet
    x_data = [x_data, x];
    y_data = [y_data, y];
    th_data = [th_data, theta];
    v_data = [v_data, v];
    w_data = [w_data, w];
    
    if t > t_max
        break;
    end
end

%% Çizim ve Simülasyon

plot(x_data, y_data, 'k--');
title('Rota ve Simülasyon');
xlabel('x (m)');
ylabel('y (m)');
hold on;
wls=[0, 0; -0.5*(L/2), 0.5*(L/2)];
wlsrot = [cos(th_data(t_max)) -sin(th_data(t_max));sin(th_data(t_max)) cos(th_data(t_max))]*wls;
h1=plot(wlsrot(1,1),wlsrot(2,1),'ro','LineWidth',2,'MarkerFaceColor','r');
h2=plot(wlsrot(1,2),wlsrot(2,2),'ro','LineWidth',2,'MarkerFaceColor','r');
h3=plot(x_data(1),y_data(1),'bo','MarkerSize',20);

% Ok ekleme
arrow_length = 0.2;
arrow_head_length = 0.7;
quiver_scale = 3;

arrow_x = arrow_length * cos(th_data(1));
arrow_y = arrow_length * sin(th_data(1));
quiver_x = x_data(1) + arrow_x;
quiver_y = y_data(1) + arrow_y;
quiver_dx = quiver_scale * arrow_x;
quiver_dy = quiver_scale * arrow_y;

quiver_arrow = quiver(quiver_x, quiver_y, quiver_dx, quiver_dy, 0, 'g', 'MaxHeadSize', arrow_head_length);

axis([-1.25 1.25 -0.5 2]);
xlim([-1 24]);
ylim([-1 1]);
t_length = length(0:dt:t)-1;
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
    pause(0.05);
end