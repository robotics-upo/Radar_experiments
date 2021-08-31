function plot_radar_stats(d)
  close all;

  fontsize = 28;
  lw = 3;
  d(:,1)=d(:,1) - d(1,1);
  hx = plot(d(:,1), d(:,2),'b', "linewidth", lw);
  hold on;
  plot(d(:,1), d(:,3),'r', "linewidth", lw);
  plot(d(:,1), d(:,4),'k', "linewidth", lw);
  x1 = xlabel ("Time (s)");
  y1 = ylabel("Number of points per measure")
  set (y1, "fontsize", fontsize) 
  set (x1, "fontsize", fontsize) 
  set(hx, "linewidth", 2);
  h=get(gcf, "currentaxes");
  set(h, "fontsize", fontsize);
end