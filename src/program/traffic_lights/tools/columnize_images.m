function [ C ] = columnize_images(X)

C = zeros(0,3);

for i = 1:size(X,1)
    img = double(cell2mat(X(i)));
    C = [C(:,1), C(:,2), C(:,3); reshape(img(:,:,1),[],1), reshape(img(:,:,2),[],1), reshape(img(:,:,3),[],1)];
end