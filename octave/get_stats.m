function [stats, hist_data]=get_stats(data, dt)
  max_time = 1e11;
  time_scale = 1e-9;
 
  for i=1:length(data)
    max_time_ = floor(data{i}(length(data{i}(:,1)),1)*time_scale);
    max_time = min([max_time, max_time_]);
    i;
  end
  n_exp = length(data) #Number of experiments
  
  printf('Max time: %d\n', max_time)
  
  stats=zeros(1, 6); # primera y segunda fila mean std_dev error m
                               # tercera y cuarta, mean std_dev error rad
                               # quinta y sexta: incertidumbre AMCL
  
  indices = ones(length(data), 1);
  
  for i=0:dt:max_time
    
    eff = 0;
    curr_data_vec = zeros(1,4);
    k = floor(i/dt);
    for j=1:length(data)
      curr_data = zeros(4,1);  
      curr_t = floor(data{j}(indices(j))*time_scale);
      %printf('Time: %d Curr_t: %d\n', i, curr_t)
      n = 0;
      
      while (floor(curr_t/dt) == k) 
        n++;
        curr_data(1,n) = data{j}(indices(j),8); # 8 es el error distancia
        curr_data(2,n) = data{j}(indices(j),9); # 9 es el error yaw
        curr_data(3,n) = sqrt( data{j}(indices(j),10) + data{j}(indices(j),11));
        curr_data(4,n) = sqrt(data{j}(indices(j),12));
        indices(j)++;
        curr_t = floor(data{j}(indices(j))*time_scale);
      endwhile
      if (n > 0)
        eff++;      
        curr_data_vec(eff, 1 )=mean(curr_data(1,:));
        curr_data_vec(eff, 2 )=mean(curr_data(2,:));  
        curr_data_vec(eff, 3 )=mean(curr_data(3,:));
        curr_data_vec(eff, 4 )=mean(curr_data(4,:));  
      end  
    end
    stats(k+1,1) = mean(curr_data_vec(:,1));
    stats(k+1,2) = std(curr_data_vec(:,1));
    stats(k+1,3) = mean(curr_data_vec(:,2));
    stats(k+1,4) = std(curr_data_vec(:,2));
    stats(k+1,5) = mean(curr_data_vec(:,3));
    stats(k+1,6) = mean(curr_data_vec(:,4));
    stats(k+1,7) = i;
    n_success = 0;
    % Calculate the percentage of success. TODO: max error?
    for j=1:length(data)
      if (curr_data_vec(j,1) < 0.5 && curr_data_vec(j,2) < 0.5)
        n_success++;
      end
        
      
    end
    stats(k+1,8) = n_success / length(data);
    
    %Get histogram:
    hist_data=curr_data_vec(:,1:2); % Get the stats of the errors in the last set of measures
  end
  
  
  
end