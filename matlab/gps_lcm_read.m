clear all;
close all;

% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath /home/lucas/my_robot/matlab/my_types.jar
 %stationary data
%  log_file = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-26-17_31_48', 'r');
%  log_file_imu = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-26-17_31_48', 'r');
%  %circle data
%  log_file = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-25-13_28_50', 'r');
%  log_file_imu = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-25-13_28_50', 'r');
 
 log_file = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-26-14_44_59', 'r');
 log_file_imu = lcm.logging.Log('/home/lucas/my_robot/log_files/lcm-log-2019-02-26-14_44_59', 'r');



% create arrays to store GPS data from log file
time = [];
lat = [];
lon = [];
alt = [];
utm_x = [];
utm_y = [];

% create arrays to store IMU data from log file 
yaw = [];
pitch = [];
roll = [];
mag_x = [];
mag_y = [];
mag_z = [];
accel_x = [];
accel_y = [];
accel_z = [];
gyro_x = [];
gyro_y = [];
gyro_z = [];

c = 1;
i = 1;

while true
 try
   evGPS = log_file.readNext();
   
   if strcmp(evGPS.channel, 'GPS')
       
      
      gps_data = lab3lcm.gps_t(evGPS.data);
     
      
      time(c) = gps_data.timestamp;
      lat(c) = gps_data.lat;
      lon(c) = gps_data.lon;
      alt(c) = gps_data.altitude;
      utm_x(c) = gps_data.utm_x;
      utm_y(c) = gps_data.utm_y;
      c = c + 1;
      
    end
  catch err   % exception will be thrown when you hit end of file
     break;
 end
 
end

while true
    try
       evIMU = log_file_imu.readNext();

       if strcmp(evIMU.channel, 'IMU')

          imu_data = lab3lcm.imu_t(evIMU.data);

          yaw(i) = imu_data.yaw;
          pitch(i) = imu_data.pitch;
          roll(i) = imu_data.roll;
          mag_x(i) = imu_data.mag_x;
          mag_y(i) = imu_data.mag_y;
          mag_z(i) = imu_data.mag_z;
          accel_x(i) = imu_data.accel_x;
          accel_y(i) = imu_data.accel_y;
          accel_z(i) = imu_data.accel_z;
          gyro_x(i) = imu_data.gyro_x;
          gyro_y(i) = imu_data.gyro_y;
          gyro_z(i) = imu_data.gyro_z;
          i = i + 1;

        end
      catch err   % exception will be thrown when you hit end of file
         break;
    end
end

% mag&gps calibration
imu_index = 0:(i-2);

std_x = std(utm_x());
std_y = std(utm_y());
mean_x = mean(utm_x);
mean_y = mean(utm_y);
adj_x = utm_x - utm_x(1);
adj_y = utm_y - utm_y(1);
end_x = adj_x(c-1);
end_y = adj_y(c-1);


%% remove zero readings from imu data and reads new data
mag_x_remove0 = mag_x(mag_x~=0);
mag_y_remove0 = mag_y(mag_y~=0);
mag_z_remove0 = mag_z(mag_z~=0);
accel_x_remove0 = accel_x(accel_x~=0);
accel_y_remove0 = accel_y(accel_y~=0);
accel_z_remove0 = accel_z(accel_z~=0);
gyro_x_remove0 = gyro_x(gyro_x~=0);
gyro_y_remove0 = gyro_y(gyro_y~=0);
gyro_z_remove0 = gyro_z(gyro_z~=0);
yaw_remove0 = yaw(yaw~=0);
pitch_remove0 = pitch(pitch~=0);
roll_remove0 = roll(roll~=0);
% mag calibration
% estimate from gyro_z and convert to degree
yaw_estimate = cumtrapz(1/40,gyro_z_remove0);
yaw_radian = yaw_remove0*pi/180;
yaw_estimate_wrap = wrapToPi(yaw_estimate)*180/pi;
yaw_unwrap = unwrap(yaw_remove0);

figure()
plot(yaw_estimate_wrap,'Linewidth',2);
hold on
plot(yaw_remove0,'Linewidth',2);
title('yaw angle estimation using yaw rate about z axis','Fontsize',24)
legend('yaw estimate','yaw from IMU','Fontsize',20)
xlabel('time','Fontsize',24)
ylabel('yaw angle (deg)','Fontsize',24)
set(gca,'FontSize',18)

%% hard & soft iron calibration in magnetometer (driving in circle)
% plot points in circle
figure()
plot(mag_x,mag_y,'Linewidth',2)
hold on
grid on
axis equal
% find ellipse parameters fit into the circle
ellipse_param = fit_ellipse(mag_x,mag_y);

% plot ellipse
t = linspace(0,2*pi,100);
theta = -ellipse_param.phi;
a=ellipse_param.a;
b=ellipse_param.b;
x0 = ellipse_param.X0_in;
y0 = ellipse_param.Y0_in;
x = x0 + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = y0 + b*sin(t)*cos(theta) + a*cos(t)*sin(theta);
plot(x,y,'LineWidth',4,'Color',[1,0,0]);
% plot major & minor axis
x1_right=x0+(a*cos(theta));
y1_right=y0+(a*sin(theta));
x1_left=x0-(a*cos(theta));
y1_left=y0-(a*sin(theta));

x2_right=x0+(b*sin(-theta));
y2_right=y0+(b*cos(theta));
x2_left=x0-(b*sin(-theta));
y2_left=y0-(b*cos(theta));

line([x1_left,x1_right],[y1_left,y1_right],'Linewidth',4,'Color',[1,0,0])
line([x2_left,x2_right],[y2_left,y2_right],'Linewidth',4,'Color',[1,0,0])

%% hard iron calibration
% translate plot
x_offset = ellipse_param.X0_in;
y_offest = ellipse_param.Y0_in;

mag_x_hi = mag_x_remove0 - x_offset;
mag_y_hi = mag_y_remove0 - y_offest;

figure
plot(mag_x_hi(1200:1800),mag_y_hi(1200:1800),'Linewidth',2)
axis equal
grid on
hold on
% translated ellipse & axis
x1 = a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y1 = b*sin(t)*cos(theta) + a*cos(t)*sin(theta);
plot(x1,y1,'LineWidth',4,'Color',[1,0,0]);

x1_right_hi=a*cos(theta);
y1_right_hi=a*sin(theta);
x1_left_hi=-a*cos(theta);
y1_left_hi=-a*sin(theta);

x2_right_hi=b*sin(-theta);
y2_right_hi=b*cos(theta);
x2_left_hi=-b*sin(-theta);
y2_left_hi=-b*cos(theta);

line([x1_left_hi,x1_right_hi],[y1_left_hi,y1_right_hi],'Linewidth',4,'Color',[1,0,0])
line([x2_left_hi,x2_right_hi],[y2_left_hi,y2_right_hi],'Linewidth',4,'Color',[1,0,0])

%% rotation
rot = (pi/2) + theta;

R = [cos(rot) sin(rot);
    -sin(rot) cos(rot)];

mag_rotate = R * [mag_x_hi(1200:1800); mag_y_hi(1200:1800)];
mag_x_rotate = mag_rotate(1,:);
mag_y_rotate = mag_rotate(2,:);

figure
plot(mag_x_rotate,mag_y_rotate,'Linewidth',2)
hold on
grid on
axis equal

% rotate ellipse
x2 = b*cos(t);
y2 = a*sin(t);
plot(x2,y2,'LineWidth',4,'Color',[1,0,0]);

line([-b,b],[0,0],'Linewidth',4,'Color',[1,0,0]);
line([0,0],[-a,a],'Linewidth',4,'Color',[1,0,0]);

%% soft iron calibration
axis_ratio = b/a;

mag_x_si = mag_x_rotate/axis_ratio;
mag_y_si = mag_y_rotate;

figure
plot(mag_x_si,mag_y_rotate,'Linewidth',2)
axis equal
grid on
hold on

ellipse_test_param = fit_ellipse(mag_x_si,mag_y_si);

x3 = a*cos(t);
y3 = a*sin(t);
plot(x3,y3,'LineWidth',4,'Color',[1,0,0]);
line([-a,a],[0,0],'Linewidth',4,'Color',[1,0,0]);
line([0,0],[-a,a],'Linewidth',4,'Color',[1,0,0]);


%% rotate back
mag_calibrated = -R*[mag_x_si;mag_y_si];
mag_x_calibrated = mag_calibrated(1,:);
mag_y_calibrated = mag_calibrated(2,:);

figure
plot(mag_x_calibrated,mag_y_calibrated,'Linewidth',2)
hold on
grid on
axis equal

x4 = a*cos(t)*cos(theta) - a*sin(t)*sin(theta);
y4 = a*sin(t)*cos(theta) + a*cos(t)*sin(theta);
plot(x4,y4,'LineWidth',4,'Color',[1,0,0]);

x3_right=a*cos(theta);
y3_right=a*sin(theta);
x3_left=-a*cos(theta);
y3_left=-a*sin(theta);

x4_right=a*sin(-theta);
y4_right=a*cos(theta);
x4_left=-a*sin(-theta);
y4_left=-a*cos(theta);

line([x3_left,x3_right],[y3_left,y3_right],'Linewidth',4,'Color',[1,0,0])
line([x4_left,x4_right],[y4_left,y4_right],'Linewidth',4,'Color',[1,0,0])

%% comparison before & after calibration
% figure
% mag_before = plot(mag_x,mag_y,'Linewidth',2);
% hold on
% grid on
% axis equal
% plot(x,y,'LineWidth',2,'Color',[1,0,0]);
% line([x1_left,x1_right],[y1_left,y1_right],'Linewidth',2,'Color',[1,0,0])
% line([x2_left,x2_right],[y2_left,y2_right],'Linewidth',2,'Color',[1,0,0])
% mag_after = plot(mag_x_calibrated,mag_y_calibrated,'Linewidth',2);
% plot(x4,y4,'LineWidth',2,'Color',[1,0,0]);
% line([x3_left,x3_right],[y3_left,y3_right],'Linewidth',2,'Color',[1,0,0])
% line([x4_left,x4_right],[y4_left,y4_right],'Linewidth',2,'Color',[1,0,0])
% title('comparison before and after HSI calibration','Fontsize',24)
% xlabel('mag_x (G)','Fontsize',24)
% ylabel('mag_y (G)','Fontsize',24)
% set(gca,'FontSize',18)
% legend([mag_before,mag_after],{'before','after'},'FontSize',20);

% yaw_estimate_mag_wrap = wrapToPi(yaw_estimate_mag);
% yaw_estimate_mag = atan2(mag_y_calibrated,-mag_x_calibrated)*180/pi;
% %complementary filter 
% yaw_filter = (0.98)*(yaw + gyro_z * 1/40) + (0.02)*(accel_x);
% figure
% plot(yaw_estimate_mag,'Linewidth',2)
% hold on
% plot(yaw_remove0,'Linewidth',2)
% hold on
% plot(yaw_estimate_wrap,'Linewidth',2)
% hold on 
% plot(yaw_filter,'Linewidth',2)
% title('yaw angle comparison','Fontsize',24)
% xlabel('time','Fontsize',24)
% ylabel('yaw angle (deg)','Fontsize',24)
% legend('yaw from mag','raw yaw','yaw from gyro','yaw filtered')
% set(gca,'FontSize',18)











% % part 6 
% %forward velocity
fs = 1;

velocity_accelerometer_estimate = cumtrapz(accel_x);
accel_xf =accel_x(1:40:60120);

time = 0:1:1502;
tim = 0:1:1501;
forward_velocity_accelerometer = cumtrapz(accel_xf)
forward_velocity_accelerometer_f = bandpass(forward_velocity_accelerometer,[0.015 0.22],fs);
%forward_velocity_accelerometer_f = cumtrapz(accel_xff)


for i = 1:1502
    vel_x(i) = utm_x(i+1)-utm_x(i);
    vel_y(i) = utm_y(i+1)-utm_y(i);
    vel(i) = sqrt((vel_x(i))^2+(vel_y(i))^2);
end

% figure(1)
% plot(time,forward_velocity_accelerometer);
% xlabel('time(s)')
% ylabel('velocity(m/s)')
% hold on;
% plot(tim,vel);
% legend('velocity-imu-corrected','velocity-gps');
% hold off;
figure(2)
plot(time,forward_velocity_accelerometer_f);
xlabel('time(s)')
ylabel('velocity(m/s)')
hold on;
plot(tim,vel);
% 
% hold on;
% plot(time,forward_velocity_accelerometer);
% legend('velocity_imu-corrected','velocity-gps','velocity-imu-orignal')
% 
% hold off;

%%


%%
%% Part7 
% 7.1
gyro_z = gyro_z(1:40:60080);
accel_y = accel_y(1:40:60080)
for i = 1:1502
    accel_y_obs(i) = vel_x(i) * gyro_z(i);
end

figure(30)
plot(accel_y_obs)
hold on
plot(accel_y)
xlabel('times(s)')
ylabel('accel_y')
legend('accel_y_obs','accel_y')

%%
figure
plot(utm_x,utm_y)

vel_y = cumtrapz(accel_y);

accel_y_sampled = accel_y;
accel_y_bpf = bandpass(accel_y_sampled,[0.015 0.22],1);
figure()
plot(accel_y)
hold on;
plot(accel_y_bpf)

vel_y_adjusted = cumtrapz(accel_y_bpf);

vehicle_position = cumtrapz(vel);
y_vehicle = cumtrapz(vel_y_adjusted);

% figure
% plot(x_vehicle,y_vehicle(1:1273))

%%
yaw_vehicle = yaw(1:40:end);

xd = zeros(1,1502);
yd = zeros(1,1502);

for i = 2:1502
    distance(i) = vehicle_position(i)-vehicle_position(i-1);
    xd(i) = xd(i-1) + sind(yaw_vehicle(i-1))*distance(i);
    yd(i) = yd(i-1) + cosd(yaw_vehicle(i-1))*distance(i);
end


figure(20)
plot(xd,yd)     % needs rotation

% % estimated plot calibration
% for i =1:1502
%     xd_calib(i) = xd(i)+1.098;
%     yd_calib(i) = yd(i)+1.02;
% end
% 
% hold on
% plot(xd_calib,yd_calib)


%% trajectory estimation
utm1 = min(utm_x);
utm2 = min(utm_y);

for i =1:1502
    utm_e_trans(i) = utm_x(i)-utm1;
    utm_n_trans(i) = utm_y(i)-utm2;
end

% figure
% plot(utm_e_trans,utm_n_trans);
% xlabel('utm-easting(m)')
% ylabel('utm-northing(m)')
% title('drive path')

% translate trajectory
for i =1:1502
    utm_e_origin(i) = utm_e_trans(i)-utm_e_trans(1);
    utm_n_origin(i) = utm_n_trans(i)-utm_n_trans(1);
end

figure
plot(utm_e_origin,utm_n_origin)
xlabel('utm-easting(m)')
ylabel('utm-northing(m)')
title('drive path')
hold on
plot(xd,yd)
axis equal

% rotate trajectory
phi = 200;
utm_rotate = [cosd(phi) -sind(phi);
              sind(phi) cosd(phi)];

route = utm_rotate*[xd;yd];
xd_calib_r = route(1,:);
yd_calib_r = route(2,:);

figure
plot(utm_e_origin,utm_n_origin)
hold on
plot(xd_calib_r,yd_calib_r)
xlabel('utm-easting(m)','Fontsize',24)
ylabel('utm-northing(m)','Fontsize',24)
title('comparison of GPS track and estimated trajectory','Fontsize',24)
legend('GPS track','estimated trajectory','Fontsize',20)

%% estimate Xc
syms xc

for i = 1:1501
    omega_der(i) = gyro_z(i+1) - gyro_z(i)
end
% xc.*omega_der = accel_y_obs - vel_x(i) * gyro_z(i)
ans = accel_y_obs - vel_x(i) * gyro_z(i)
ans_c = bandpass(ans,[0.3 0.4],1)
omega_der = 0.5.*omega_der
figure
plot(ans_c)
hold on 
plot(omega_der)
legend('ans','xc*omega_der')

%%
% lon1 = lon(lon~=0);
% lat1 = lat(lat~=0);
% alt1 = alt(alt~=0);
% a = min(lon1);
% b = min(lat1);
% 
% for i =1:1275
%     u1(i) = lon1(i)-a;
%     u2(i) = lat1(i)-b;
% end
% 
% figure
%     plot(u1,u2,'-');
%     xlabel('utm-easting(m)')
%     ylabel('utm-northing(m)')
%     title('drive path')





%%

% figure(1);
% plot(adj_x, adj_y,'*');
% title('UTM Plot');
% xlabel('UTM X Displacement from Start (m)');
% ylabel('UTM Y Displacement from Start (m)');
%axis([.00 .3 .05 .3]);

% 
% figure(2);
% plot(imu_index, yaw);
% title('Yaw vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Yaw (degrees)');
% % 
% figure(3);
% plot(imu_index, pitch);
% title('Pitch vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Pitch (degrees)');
% % 
% figure(4);
% plot(imu_index, roll);
% title('Roll vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Roll (degrees)');
% % 
% figure(5);
% plot(imu_index, mag_x);
% title('Mag. X vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Mag. X');
% % 
% figure(6);
% plot(imu_index, mag_y);
% title('Mag. Y vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Mag. Y');
% % 
% figure(7);
% plot(imu_index, mag_z);
% title('Mag. Z vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Mag. Z');
% % 
% figure(8);
% plot(imu_index, accel_x);
% title('Accel. X vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Accel. X (m/s^2)');
% % 
% figure(9);
% plot(imu_index, accel_y);
% title('Accel. Y vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Accel. Y (m/s^2)');
% % % 
% figure(10);
% plot(imu_index, accel_z);
% title('Accel. Z vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Accel. Z (m/s^2)');
% % 
% figure(11);
% plot(imu_index, gyro_x);
% title('Gyro X vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Gyro X (deg/s)');
% 
% figure(12);
% plot(imu_index, gyro_y);
% title('Gyro Y vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Gyro Y (deg/s)');
% 
% figure(13);
% plot(imu_index, gyro_z);
% title('Gyro Z vs. Data Point Index');
% xlabel('Data Point Index');
% ylabel('Gyro Z (deg/s)');

% %figure(14);
% plot(mag_x,mag_y,'*','Markersize',1);
% title('Magnetometer');
% xlabel('mag x');
% ylabel('mag y');
% axis([.00 .2 .00 .2]);
% % 
% figure(15);
% plot(mag_xc,mag_yc,'o','Markersize', 1);
% title('Magnetometer: Hard and Soft Iron Adjusted');
% xlabel('mag x compensated');
% ylabel('mag y compensated');
% axis([0 .1 0 .1]);

% figure(16);
% plot(utm_x,utm_y, '*', 'Markersize',1);
% title('Utm');
% xlabel('utm_x');
% ylabel('utm_y');
%axis([])
