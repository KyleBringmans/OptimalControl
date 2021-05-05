function result = median_filter(data, order)
% median filter of the chosen order
data_size = size(data);
if (data_size(2) >= order)
    data_segment = data(end-order+1:end);
    sorted_data = sort(data_segment);
    middle_element = floor(order/2) + 1;
    result = sorted_data(middle_element);
else
    result = data(end);
end
end