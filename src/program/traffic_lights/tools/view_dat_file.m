function view_ssd_dat_file(filename)

f = fopen(filename);
s1 = fread(f, 1, 'int32');
s2 = fread(f, 1, 'int32');
t = fread(f, s1*s2, 'single');
t = reshape(t, s1, []);
imagesc(t');

end

