function [ X ] = load_images(path, file)

files = dir(fullfile(path, file));
X = cell(size(files,1),1);
for i = 1:size(X,1)
    X(i) = {imread(fullfile(path,files(i).name))};
end