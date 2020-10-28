%% Function exportData.m
% Export cell array to desired folder. Filename concatenated with datetime
% string at time of running.
% Name format: mm-dd-yyyy HH-MM-simulationName.mat

%% To do
% - create new folder for all saved variables seeing as they are saved
% independently?
% - create plot function - difficult as will completely depend on input data
% - test function
% - move plotting function outside of this script?
% - create "data" file within save directory? - and add the datetime to this
% folder name instead of using on the filenames.

%% Function save data
function [] = exportData(data, saveDir, folder)

  % Save working directory path
  workingDir = pwd;
  % Change to save directory
  cd(saveDir); 
  % Create save directory
  dateTime = datestr(now,'yyyy-mm-dd-HH-MM');
  if folder ~= ""
    folder = strcat(dateTime, '-', folder);
  else
    folder = dateTime;
  end
  mkdir(folder);
  cd(folder);

  % Save variables
  for i=1:length(data)
    varName = data{i, 1};
    varData = data{i, 2};
    % Save
    if data{i, 3} == true
      saveFile = strcat(varName, '.mat');
      save(saveFile, 'varData');
    end
  end

  % Go back to working directory
  cd(workingDir);
end