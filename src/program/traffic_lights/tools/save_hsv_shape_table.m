function save_hsv_shape_table( prefix, type, h, a )

ind1 = find(h, 1, 'first');
ind2 = find(h, 1, 'last');
h = h(ind1:ind2);

filename = sprintf('%s.%s', prefix, type);
f = fopen(filename, 'w');
fwrite(f, [ind1;ind2], 'uint16');
fwrite(f, a, 'double');
fwrite(f, h, type);

end

