function stats=get_stats(data)
  max_time = 1e10
  time_scale = 1e-10
  for i=1:length(data)
    max_time_ = floor(data{i}(length(data{i}(:,1)),1)*time_scale);
    max_time = min([max_time, max_time_])
    i
  end
  
  printf('Max time: %d\n', max_time)
  
  stats=zeros(max_time + 1, 4); # primera y segunda fila mean std_dev error m
                               # tercera y cuarta, mean std_dev error rad
  
  indices = ones(length(data), 1);
  
  for i=0:max_time
    
    eff = 0;
    curr_data_vec = zeros(1,2);
    for j=1:length(data)
      curr_data = zeros(2,1);  
      curr_t = floor(data{j}(indices(j))*time_scale);
      printf('Time: %d Curr_t: %d\n', i, curr_t)
      n = 0;
      while (curr_t == i) 
        n++;
        curr_data(1,n) = data{j}(indices(j),8); # 8 es el error distancia
        curr_data(2,n) = data{j}(indices(j),9); # 9 es el error yaw
        indices(j)++;
        curr_t = floor(data{j}(indices(j))*time_scale);
      endwhile
      if (n > 0)
        eff++;      
        curr_data_vec(eff, 1 )=mean(curr_data(1,:));
        curr_data_vec(eff, 2 )=mean(curr_data(2,:));  
      end  
    end
    stats(i+1,1) = mean(curr_data_vec(:,1));
    stats(i+1,2) = std(curr_data_vec(:,1));
    stats(i+1,3) = mean(curr_data_vec(:,2));
    stats(i+1,4) = std(curr_data_vec(:,2));
  end
  
end