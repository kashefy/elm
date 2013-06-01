% load MNIST images from mat file and generate csv files

strInputPath = 'C:\Users\woodstock\Documents\grad\Thesis\code\matlab\SEM\Data\MNIST_digits\';
strInputFilename = 'MNIST.mat';

strOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\';
strOutputFilename_metaData  = 'metaData.csv';
strOutputFilename_vals      = 'values.csv';
strOutputFilename_labels    = 'labels.csv';

fid_labels = fopen([ strOutputPath strOutputFilename_labels ], 'w');

load([strInputPath strInputFilename]);
% filter out classes
%classesToInclude = [4, 7, 1, 2, 3, 5, 6, 8, 9, 0];
classesToInclude = [0, 1, 2, 4];
nofClassesToInclude = length(classesToInclude);
includeIndicies = zeros(1, length(y));
for i=1:nofClassesToInclude
    includeIndicies = includeIndicies | y == classesToInclude(i);
end
includeIndicies = find(includeIndicies);
X = X(:,includeIndicies);
y = y(includeIndicies);
[ nofPixels nofImages ] = size(X);

nofImages = 2000;
imgDims = floor(sqrt(nofPixels));
imgMode = 0; % binary
%imgMode = 1; % [0,1]
%imgMode = 255; % [0,255]
nofChannels = 1; % 1 => grayscale, 3 => RGB


%% write metaData
fid_metaData = fopen([ strOutputPath strOutputFilename_metaData ], 'w');
fprintf(fid_metaData,'%s,',['n']);
fprintf(fid_metaData,'%i', nofImages);
fprintf(fid_metaData,'\n');
fprintf(fid_metaData,'%s,',['r']);
fprintf(fid_metaData,'%i', imgDims);
fprintf(fid_metaData,'\n');
fprintf(fid_metaData,'%s,',['c']);
fprintf(fid_metaData,'%i', imgDims);
fprintf(fid_metaData,'\n');
fprintf(fid_metaData,'%s,',['m']);
fprintf(fid_metaData,'%i', imgMode);
fprintf(fid_metaData,'\n');
fprintf(fid_metaData,'%s,',['h']);
fprintf(fid_metaData,'%i', nofChannels);
fprintf(fid_metaData,'\n');
fprintf(fid_metaData,'%s,',['l']);
fprintf(fid_metaData,'%i', nofClassesToInclude);
fprintf(fid_metaData,'\n');
fclose(fid_metaData);

% write values
filename = [strOutputPath  strOutputFilename_vals ];
M = [];
if(imgMode == 0)
    
    M = ((X(:,1:nofImages)/255)>0.5)';   % binary values for pixels 
    
elseif (imgMode == 1)

    M = (X(:,1:nofImages)/255)';   % [0,1] range for pixels
        
elseif (imgMode == 255)

    M = X(:,1:nofImages)';       % [0,255] range for pixels 
            
end
dlmwrite(filename, M, 'newline', 'pc');

% write labels
filename = [strOutputPath  strOutputFilename_labels ];
My = y(1:nofImages)';
dlmwrite(filename, My, 'newline', 'pc');

%%

nofPlotRows = ceil(sqrt(nofImages));
nofPlotCols = ceil(sqrt(nofImages));
nofImagesToPlot = nofPlotRows*nofPlotCols;

if (nofImages>50)
    disp('too many to plot')
else
    figure(1)
    for i=1:nofImages

        imVec = M(i,:);
        %dims = floor(sqrt(length(imVec)));
        %img = reshape(imVec,dims,dims);
        img = reshape(imVec,imgDims,imgDims);
        img = img';

        imgBinary = img;% > 0.5;

        subplot(nofPlotRows,nofPlotCols,i)
        imagesc(imgBinary);

    end
end

%%
% figure(2)
% for i=1:5
%     f = [strOutputPath,num2str(i-1),'.csv'];
%     s=dlmread(f);
%     s=reshape(s,28,28);
%     subplot(nofPlotRows,nofPlotCols,i)
%     imagesc(s);
% end


