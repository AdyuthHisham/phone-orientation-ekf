function startup()
% STARTUP  Add the course-provided sensordata.jar (place it in src/matlab/live/lib/) to the Java path.
  jar = fullfile(fileparts(mfilename('fullpath')), 'lib', 'sensordata.jar');
  fprintf('Loading: %s\n', jar);
  javaaddpath(jar);
end
