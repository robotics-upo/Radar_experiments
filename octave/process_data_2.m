close all % Close all figures and start it over!

% Parameters
fontsize = 18;
#experiment='_12_';   # or ...
#experiment = '_27_nov_13_';
experiment = '_27_nov_12_14_';
n_max = 5;
lw = 2;
dt = 20;

left = 0.15;
width = 0.7;

plots = 4;

margin = 0.1;
mini_margin = 0.02;
v_size = (1 - 2*margin - plots * mini_margin) / 4;

pos = 3*(v_size + mini_margin) + margin;
hax1 = subplot ("position", [left pos width v_size]);

addpath('/home/chur/fiducial_ws/src/Radar_experiments/octave');
cd ../stats
data_dbscan = load_data(1,n_max,strcat('stats_dbscan_lines',experiment));


a = get_stats(data_dbscan, dt);
plot(a(:,7), a(:,1), "linewidth", lw);
data_lidar = load_data(1,n_max,strcat('stats_lidar',experiment));
b = get_stats(data_lidar, dt);
hold on;
plot(b(:,7), b(:,1),'g', "linewidth", lw);

data_radar = load_data(1,n_max,strcat('stats_radar',experiment));
c = get_stats(data_radar, dt);
plot(c(:,7), c(:,1),'r', "linewidth", lw);
axis("labely");
hy1 = ylabel("Position error (m)");
set (hy1, "fontsize", fontsize) ;

printf("Mean position errors: %f, %f, %f\n", mean(a(:,1)), mean(b(:,1)), mean(c(:,1)))

%figure(2)
pos = 2*(v_size + mini_margin) + margin;
hax2 = subplot ("position", [left pos width v_size]);
plot(a(:,7), a(:,2),'b', "linewidth", lw);
hold on;
plot(b(:,7), b(:,2),'g', "linewidth", lw);
plot(c(:,7), c(:,2),'r', "linewidth", lw);
axis("labely");
hy2 = ylabel("Orientation error (rad)");
set (hy2, "fontsize", fontsize);

printf("Mean orientation errors: %f, %f, %f\n", mean(a(:,2)), mean(b(:,2)), mean(c(:,2)))

% Plot uncertainty AMCL dist
pos = 1*(v_size + mini_margin) + margin;
hax3 = subplot ("position", [left pos width v_size]);
plot(a(:,7), a(:,3),'b', "linewidth", lw);
hold on;
plot(b(:,7), b(:,3),'g', "linewidth", lw);
plot(c(:,7), c(:,3),'r', "linewidth", lw);
axis("labely");
hy3 = ylabel("AMCL uncert. (m)");
set (hy3, "fontsize", fontsize);

printf("Mean AMCL uncertainty: %f, %f, %f\n", mean(a(:,3)), mean(b(:,3)), mean(c(:,3)));

hax4 = subplot ("position", [left margin width v_size]);
plot(a(:,7), a(:,4),'b', "linewidth", lw);
hold on;
plot(b(:,7), b(:,4),'g', "linewidth", lw);
plot(c(:,7), c(:,4),'r', "linewidth", lw);
hx1 = xlabel ("Time (s)");
set (hx1, "fontsize", fontsize) 
hy4 = ylabel("AMCL uncert. (rad)")
linkaxes ([hax1, hax2, hax3, hax4],"x");
set (hy4, "fontsize", fontsize) 
set(hax4, "fontsize", fontsize, "linewidth", 2);
set(hax3, "fontsize", fontsize, "linewidth", 2);
set(hax2, "fontsize", fontsize, "linewidth", 2);
set(hax1, "fontsize", fontsize, "linewidth", 2);

printf("Mean AMCL uncertainty: %f, %f, %f\n", mean(a(:,4)), mean(b(:,4)), mean(c(:,4)))

printf("Success rate a:\n")
a(:,8)
printf("Success rate b:\n") 
b(:,8)
printf("Success rate c:\n") 
c(:,8)

printf("End\n")

