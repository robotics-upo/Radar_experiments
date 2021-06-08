function data=load_data(n_min, n_max, base_name)
  data = cell(n_max);
  for i=n_min:n_max
    name = strcat(base_name, mat2str(i), '.txt');
    data{i} = load (name);
    data{i}(:,1) = data{i}(:,1) - data{i}(1,1); # Init at 0 s
  end
  
end
  