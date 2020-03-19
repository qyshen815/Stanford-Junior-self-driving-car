function create_hsv_tables(path)

R = load_images(fullfile(path, 'red_lights'), '*.png');
G = load_images(fullfile(path, 'grn_lights'), '*.png');
Y = load_images(fullfile(path, 'ylw_lights'), '*.png');

hsvR = columnize_images(R);
hsvG = columnize_images(G);
hsvY = columnize_images(Y);

hue_hist_R = hist(hsvR(:,3), 0:255);
hue_hist_G = hist(hsvG(:,3), 0:255);
hue_hist_Y = hist(hsvY(:,3), 0:255);

hue_hist_R = uint8(255 * hue_hist_R / max(max(hue_hist_R)));
hue_hist_G = uint8(255 * hue_hist_G / max(max(hue_hist_G)));
hue_hist_Y = uint8(255 * hue_hist_Y / max(max(hue_hist_Y)));

figure; 
bar(hue_hist_R, 'r');
hold on;
bar(hue_hist_G, 'g');
bar(hue_hist_Y, 'y');
hold off;

save_hsv_shape_table('red_hue_shape_table', 'uint8', hue_hist_R, 255);
save_hsv_shape_table('grn_hue_shape_table', 'uint8', hue_hist_G, 255);
save_hsv_shape_table('ylw_hue_shape_table', 'uint8', hue_hist_Y, 255);

sat = [ hsvR(:,2); hsvG(:,2); hsvY(:,2) ];
sat_hist = hist(sat, 0:255);
save_hsv_shape_table('saturation_shape_table', 'uint8', sat_hist, 255);
