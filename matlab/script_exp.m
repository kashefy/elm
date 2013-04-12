 close all
 modelOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\';
 setDir = 'MNIST\\tune_0\\';
 
 filename = 'masksLearners_layerF.csv';
 [arr_masks_learners_layerF, arr_masks_singles_layerF] = load_masks(fullfile(modelOutputPath, setDir, filename), fullfile(modelOutputPath, setDir, 'activity_layerF'));
 display_masks(arr_masks_learners_layerF, 2000, arr_masks_singles_layerF);
 % polarize mask value using threshold
 for i = 1:length(arr_masks_learners_layerF)
     arr_masks_learners_layerF{i} = arr_masks_learners_layerF{i} > 0.2;
 end
 
 filename = 'weights_layerF.csv';
 nof_orientations = 12;
 nof_intensities = 2;
 dims = [9, 9];
 slicing = {[prod(dims)*nof_orientations, nof_orientations], [prod(dims)*nof_intensities, nof_intensities]};
 weights = read_weights(fullfile(modelOutputPath, setDir, filename), slicing);
 display_weights_orient(weights{1}, 4000, arr_masks_learners_layerF);
 
 display_weights_intensity(weights{2}, 4002);
 

 filename = 'response1D_layerF_label_learn.csv';
 filepath_labels = fullfile(modelOutputPath, setDir, filename);
 filename = 'response1D_layerF_learn.csv';
 filepath_responses_layerF = fullfile(modelOutputPath, setDir, filename);
 filename = 'response1D_layerZ_learn.csv';
 filepath_responses_layerZ = fullfile(modelOutputPath, setDir, filename);
 display_response_and_label(filepath_labels, filepath_responses_layerF, 5000, filepath_responses_layerZ);

%% layerF
 modelOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\';
 setDir = 'MNIST\\tune_tune_0\\';
 % visualize learner response
 filename = 'response.csv';
 nofCauses = 2;
 %drawResponseHist([modelOutputPath, setDir,  filename], nofCauses);
 
  % visualize prediction stats
filename = 'predictionStats_layerF.csv';
[ firingProbs, condEntropyFinal, preferredCause] = loadPredictionStats([modelOutputPath, setDir,  filename]);
displayPredictionStats(firingProbs, condEntropyFinal, 500, 'predictionStats_layerF');
filename = 'predictionStats_F2Z.csv';
[ firingProbs, condEntropyFinal, preferredCause] = loadPredictionStats([modelOutputPath, setDir,  filename]);
displayPredictionStats(firingProbs, condEntropyFinal, 600, 'predictionStats_F2Z');

watch_dir = 'watch\\';
filename  = 'watchAvgCondEntropy_perClass_layerF.csv';
displayAvgCondTenropy([modelOutputPath, setDir,  watch_dir, filename], 700, 'watch avgCondEntropy perClass layerF');
filename  = 'watchAvgCondEntropy_perAttWin_layerF.csv';
displayAvgCondTenropy([modelOutputPath, setDir,  watch_dir, filename], 710, 'watch avgCondEntropy perAttWin layerF');
filename  = 'watchAvgCondEntropy_layerZ.csv';
displayAvgCondTenropy([modelOutputPath, setDir,  watch_dir, filename], 720, 'watch avgCondEntropy layerZ');

 % visualize acitvity
%  filename = 'masksClasses.csv';
%  figure(701)
%  arrMasksClasses = displayMasks([modelOutputPath, setDir,  filename]);
%  arrPreMasks = arrMasksClasses;
%  for i=1:length(preferredCause)
%      arrMasksClasses{i} = arrPreMasks{preferredCause(i)};
%  end
 filename = 'masksLearners_layerF.csv';
 figure(705)
 [arr_masks_learners_layerF, arr_masks_singles_layerF] = displayMasks([modelOutputPath, setDir, filename], [modelOutputPath, setDir, 'activity_layerF\\']);
 filename = 'masksLearners_layerZ.csv';
 figure(702)
 [arr_masks_learners_layerZ, arr_masks_singles_layerZ] = displayMasks([modelOutputPath, setDir, filename], [modelOutputPath, setDir, 'activity_layerZ\\']);
 % save mask + singles per learner to image file
%  nof_masks = length(arr_masks_learners_layerF)
%  [~, nof_singles]= size(arr_masks_singles_layerF)
%  clims = [0, 1];
%  for mi = 1:nof_masks
%      
%      figure(707)
%      subplot(1,1+nof_singles,1)
%      imagesc(arr_masks_learners_layerF{mi}, clims)
%      for si = 1:nof_singles
%          
%          if ~isempty(arr_masks_singles_layerF{mi, si})
%              subplot(1,1+nof_singles,1+si)
%          end
%      end
%      
%      subplot(1, 1+
%      
%      
%      close figure 707
%      
%  end
 
 % visualize weights
 nofFeatureSets = 6;
 %filename = 'weights.csv';
 %displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets);
% displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasks);
%displayWeights2([modelOutputPath, setDir,  filename], nofFeatureSets, arr_masks_learners, 4000);
% filename = 'weightsAux.csv';
%displayWeights2([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasksLearners,2000);
 %filename = 'weights_layerF.csv';
 %displayWeights2([modelOutputPath, setDir,  filename], nofFeatureSets, arr_masks_learners_layerF, 4000);

%% visualize attention samples
strPath = 'C:\Users\woodstock\Documents\grad\Thesis\code\repo\root_visionModel\trunk\ModelFrontEnd\data\output\MNIST\';

figureOffset = 7000;
displayAttention(strPath, [], figureOffset);

strSuffix = 'All';
figureOffset = 8000;
displayAttention(strPath, strSuffix, figureOffset);
    

%% weight and rate watch
str_main_dir = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\';
str_set_dir = 'MNIST';
str_watch_dir = 'watch';
figure_offset_weight_watch = 3000;

arr_weight_watch = [];
arr_weight_watch_layerF = [];

for i_learner = 0:100
    
    str_weight_watch_filename = ['weightWatch', num2str(i_learner), '.csv'];
    str_weight_watch_filepath = fullfile(str_main_dir, str_set_dir, str_watch_dir, str_weight_watch_filename);
    
    status = exist(str_weight_watch_filepath, 'file');
    
    if status == 2 % 2 == name is the full pathname to any file
        
        weight_watch_i = dlmread(str_weight_watch_filepath);
        arr_weight_watch = [arr_weight_watch, weight_watch_i'];
    end
    
    % layerF
    str_weight_watch_filename = ['weightWatch_layerF', num2str(i_learner), '.csv'];
    str_weight_watch_filepath = fullfile(str_main_dir, str_set_dir, str_watch_dir, str_weight_watch_filename);
    
    status = exist(str_weight_watch_filepath, 'file');
    
    if status == 2 % 2 == name is the full pathname to any file
        
        weight_watch_i = dlmread(str_weight_watch_filepath);
        arr_weight_watch_layerF = [arr_weight_watch_layerF, weight_watch_i'];
    end
end

figure(figure_offset_weight_watch);
subplot(2,1,1)
plot(arr_weight_watch);
title('weightWatch');

figure(figure_offset_weight_watch+10);
subplot(2,1,1)
plot(arr_weight_watch_layerF);
title('weightWatch layerF');



 %% visualize filter response
% strPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\barSet\\filterResponse\\'; 
% displayFilterResponse(strPath);
% %%
% 
%strPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\barNoiseSet\\filterResponse\\'; 
%displayFilterResponse(strPath);
% %%
% 
% strPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\filterResponse\\'; 
% displayFilterResponse2(strPath);

% strPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\filterResponse\\'; 
% displayFilterResponse3(strPath);

strPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\filterResponse\\'; 
displayFilterResponse4(strPath);


%%
 %modelInputPath = 'C:\\Users\\woodstock\\Documents\\NetBeansProjects\\ModelFrontEnd\\output\\';
 modelOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\';
 setDir = 'barSet\\';
 % visualize learner response
 filename = 'response.csv';
 nofCauses = 6;
 %drawResponseHist([modelOutputPath, setDir, filename], nofCauses);
 
  % visualize prediction stats
filename = 'predictionStats.csv';
[ firingProbs, condEntropyFinal, preferredCause] = loadPredictionStats([modelOutputPath, setDir,  filename]);
displayPredictionStats(firingProbs, condEntropyFinal);
filename  = 'avgCondEntropy.csv';
displayAvgCondTenropy([modelOutputPath, setDir,  filename]);
 
  % visualize acitvity
 filename = 'masks.csv';
 arrMasks = displayMasks([modelOutputPath, setDir, filename]);
 arrPreMasks = arrMasks;
 for i=1:length(preferredCause)
     if(preferredCause(i)==0)
         [r,c] = size(arrPreMasks{1}); % just need the size 
         arrMasks{i} = ones(r,c);   % uncover all
     else
         arrMasks{i} = arrPreMasks{preferredCause(i)};
     end
     
 end
 
 % visualize weights
 nofFeatureSets = 6;
 filename = 'weights.csv';
 displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasks);
  
 %%
 modelOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\';
 setDir = 'MNIST\\';
 % visualize learner response
 filename = 'response.csv';
 nofCauses = 2;
 %drawResponseHist([modelOutputPath, setDir,  filename], nofCauses);
 
  % visualize prediction stats
filename = 'predictionStats.csv';
[ firingProbs, condEntropyFinal, preferredCause] = loadPredictionStats([modelOutputPath, setDir,  filename]);
displayPredictionStats(firingProbs, condEntropyFinal, 100, 'predictionStats');
filename  = 'avgCondEntropy.csv';
displayAvgCondTenropy([modelOutputPath, setDir,  filename], 150, 'avgCondEntropy');

 % visualize acitvity
 filename = 'masksClasses.csv';
 figure(701)
 arrMasksClasses = displayMasks([modelOutputPath, setDir,  filename]);
 arrPreMasks = arrMasksClasses;
 for i=1:length(preferredCause)
     arrMasksClasses{i} = arrPreMasks{preferredCause(i)};
 end
 filename = 'masksLearners.csv';
 figure(702)
 arrMasksLearners = displayMasks([modelOutputPath, setDir,  filename], [modelOutputPath, setDir,  'activity\\']);
 
 % visualize weights
 nofFeatureSets = 6;
 filename = 'weights.csv';
 %displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets);
% displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasks);
displayWeights2([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasksLearners,4000);
% filename = 'weightsAux.csv';
%displayWeights2([modelOutputPath, setDir,  filename],nofFeatureSets,arrMasksLearners,2000);



%%
% visualize aux neurons
 modelOutputPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo\\root_visionModel\\trunk\\ModelFrontEnd\\data\\output\\';
 setDir = 'MNIST\\';
  nofFeatureSets = 6;
 filename = 'weightsAux.csv';
 displayWeights([modelOutputPath, setDir,  filename],nofFeatureSets);
%%


% create bar stimulus
a=zeros(101);
[nofRows,nofCols]=size(a);
yOrigin = ceil(nofCols/2);
xOrigin = ceil(nofRows/2);
angle = degtorad(-45);
band = 1;
if band == 1 || mod(band,2)
    evenBand = 0;
else
    evenBand = 1;
end
a(yOrigin,xOrigin) = 1;

for x = 1:nofCols
    dx = x-xOrigin;
    y = floor(yOrigin - dx*tan(angle));

    for bx=-floor(band/2)+x:floor(band/2)+x-evenBand;
        for by=-floor(band/2)+y:floor(band/2)+y-evenBand;
            if by > 0 && by <= nofRows && bx > 0 && bx <= nofCols
                a(by,bx) = 1;
            end
        end
    end
end
imagesc(a);
%%

path = 'C:\\Users\\woodstock\\Documents\\NetBeansProjects\\ModelFrontEnd\\';
re = dlmread([path 're.csv']);
im = dlmread([path 'im.csv']);
figure(6)
subplot(1,2,1)
imagesc(re);
axis equal
axis tight
subplot(1,2,2)
imagesc(im);
axis equal
axis tight

fre = dlmread([path 'fre.csv']);
fim = dlmread([path 'fim.csv']);
figure(8)
subplot(2,2,1)
imagesc(fftshift(fre));
axis equal
axis tight
subplot(2,2,2)
imagesc(fftshift(fim));
axis equal
axis tight

fre = dlmread([path 'fre2.csv']);
fim = dlmread([path 'fim2.csv']);
figure(8)
subplot(2,2,3)
imagesc(fftshift(fre));
axis equal
axis tight
subplot(2,2,4)
imagesc(fftshift(fim));
axis equal
axis tight

%% visualizing results from testEncodeViaPro
i = [0.6937188393550658  0.9083248697648544  0.5396485278937965  0.5998512727869267  0.5001344841471391  0.38711628303155365];
i = i/max(i);
h = [17492  19670  16060  16577  15621  14580 ];
h = h/max(h);
figure(111);
subplot(2,1,1);
bar(i);
subplot(2,1,2);
bar(h)

%%
%x = [3.098269218580311E-10 1.248070143686117E-7 3.405061763217425E-9 6.763039550193427E-11 9.999748730981958E-8 3.0201030877006185E-9];
%x = [1.2376063627973997 2.260230589253699 2.260587291932708 1.235234713410862 0.3870794133625845 0.39263851173339565 ];5
%x = [0.07927221692384918 0.04984124376386921 0.015971820262816294 0.0014643572544827143 0.0014076373300973412 0.035399886811711426  ];
%x = [1.0697076574795819 0.7368882678615889 0.17318164257901997 0.08139855913315681 0.17262206108655317 0.7369376611652151 ];
%x = [4.567470136688176E-9 1.6359883028284774E-6 1.7011253850015092E-4 7.760779178723009E-5 5.483669862205622E-7 1.7023024698497142E-9];

x = [1.2026687403640692 0.8368796100167223 0.6696070686280386 0.5261017603365263 0.7979110326020376 1.3122596468825884];
subplot(3,1,1);
bar(x);
p = x/sum(x);
var(p)
beta = 0.3/var(p);
y = exp(beta.*p);
z = sum(y);
y = y./z;
subplot(3,1,2);
bar(y);
beta = 0.1;
y = exp(beta.*p);
z = sum(y);
y = y./z;
subplot(3,1,3);
bar(y);

%%
x=[0.8465944938486786 0.23263214476485744 0.209772170663834 0.3908406649128744 0.7687986725959902 1.1030135374156098 ];
subplot(3,1,1);
bar(x);
y=[0.0691464516963949 1.63583489300608E-4 1.3312873950582034E-4 7.395980779141707E-4 0.031769802489948644 0.8980474355069359 ];
subplot(3,1,2);
bar(y);
z=[5 5 5 5 5 5 5 5 5 5 5 5 5 5 0 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 ];
subplot(3,1,3);
hist(z+1,[1:6]);

%%
figure(1);
subplot(2,1,1);
imagesc(dlmread('C:\Users\woodstock\Documents\grad\Thesis\code\repo\root_visionModel\trunk\ModelFrontEnd\a.txt'));
subplot(2,1,2);
imagesc(dlmread('C:\Users\woodstock\Documents\grad\Thesis\code\repo\root_visionModel\trunk\ModelFrontEnd\b.txt'));