function stats=get_stats(data, dt)
  max_time = 1e11
  time_scale = 1e-9
 
  for i=1:length(data)
    max_time_ = floor(data{i}(length(data{i}(:,1)),1)*time_scale);
    max_time = min([max_time, max_time_])
    i
  end
  
  printf('Max time: %d\n', max_time)
  
  stats=zeros(max_time + 1, 6); # primera y segunda fila mean std_dev error m
                               # tercera y cuarta, mean std_dev error rad
  
  indices = ones(length(data), 1);
  
  for i=0:max_time:dt
    
    eff = 0;
    curr_data_vec = zeros(1,4);
    for j=1:length(data)
      curr_data = zeros(4,1);  
      curr_t = floor(data{j}(indices(j))*time_scale/dt);
      %printf('Time: %d Curr_t: %d\n', i, curr_t)
      n = 0;
      while (curr_t == i) 
        n++;
        curr_data(1,n) = data{j}(indices(j),8); # 8 es el error distancia
        curr_data(2,n) = data{j}(indices(j),9); # 9 es el error yaw
        curr_data(3,n) = sqrt( data{j}(indices(j),10)*data{j}(indices(j),10) + data{j}(indices(j),11)*data{j}(indices(j),11));
        curr_data(4,n) = data{j}(indices(j),12);
        
        indices(j)++;
        curr_t = floor(data{j}(indices(j))*time_scale);
      endwhile
      if (n > 0)
        eff++;      
        curr_data_vec(eff, 1 )=mean(curr_data(1,:));
        curr_data_vec(eff, 2 )=mean(curr_data(2,:));  
        curr_data_vec(eff, 3 )=mean(curr_data(1,:));
        curr_data_vec(eff, 4 )=mean(curr_data(2,:));  
      end  
    end
    stats(i+1,1) = mean(curr_data_vec(:,1));
    stats(i+1,2) = std(curr_data_vec(:,1));
    stats(i+1,3) = mean(curr_data_vec(:,2));
    stats(i+1,4) = std(curr_data_vec(:,2));
    stats(i+1,5) = mean(curr_data_vec(:,3));
    stats(i+1,6) = mean(curr_data_vec(:,4));
  end
  
end