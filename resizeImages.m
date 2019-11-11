x = imread('./submarine.png');
%y = imresize(x, [40, 100]); %destroyer
%y = imresize(x, [200, 200]); %submarine
y = imresize(x, [20, 60]);
imwrite(y, './subsmall.png');