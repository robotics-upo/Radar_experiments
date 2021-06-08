close all % Close all figures and start it over!
figure(1);
dt = 20
addpath('/home/chur/fiducial_ws/src/Radar_experiments/octave');
cd ../stats
data_dbscan = load_data(1,10,'stats_dbscan_fusion_11_');
a = get_stats(data_dbscan);
plot(a(:,1));-+
data_lidar = load_data(1,10,'stats_lidar_11_');
b = get_stats(data_lidar);
hold on;
plot(b(:,1),'g');
data_fusion = load_data(1,10,'stats_fusion_11_');
c = get_stats(data_fusion);
plot(c(:,1),'r');
data_radar = load_data(1,10,'stats_radar_11_');
d = get_stats(data_radar);
plot(d(:,1),'k');

figure(2)
plot(a(:,2),'b');
hold on;
plot(b(:,2),'g');
plot(c(:,2),'r');
plot(d(:,2),'k');


% Plot uncertainty AMCL dist
figure(3)
plot(a(:,3),'b');
hold on;
plot(b(:,3),'g');
plot(c(:,3),'r');
plot(d(:,3),'k');

figure(4)
plot(a(:,4),'b');
hold on;
plot(b(:,4),'g');
plot(c(:,4),'r');
plot(d(:,4),'k');