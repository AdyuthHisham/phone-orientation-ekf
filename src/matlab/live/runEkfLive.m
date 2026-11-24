function [xhat, meas] = runEkfLive(port)
% RUNEKFLIVE  Filter smartphone sensor data in real time and show a live 3D view.
%   Needs the course-provided pieces (not redistributed here): sensordata.jar on the Java path
%   (see startup.m) and OrientationView.m on the MATLAB path. Start this function first, then
%   start streaming from the phone (Sensor Fusion app) to this computer's IP, port 3400.
%   Returns the estimates xhat (t, x, P) and the raw measurements meas, incl. the phone's own
%   orientation. Not run in this repo: no phone stream was available.
  if nargin < 1, port = 3400; end
  import('com.liu.sensordata.*');
  cfg = ekfConfig();
  s = struct('x', cfg.x0, 'P', eye(4), 'L', cfg.L0, 'tPrev', NaN);
  xhat = struct('t', zeros(1,0), 'x', zeros(4,0), 'P', zeros(4,4,0));
  meas = struct('t', zeros(1,0), 'acc', zeros(3,0), 'gyr', zeros(3,0), 'mag', zeros(3,0), 'orient', zeros(4,0));
  t0 = [];
  try
    server = StreamSensorDataReader(port);
    sentinel = onCleanup(@() server.stop());
    server.start();
    figure(1); subplot(1, 2, 1); ownView = OrientationView('Own filter', gca);
    googleView = []; counter = 0;
    while server.status()
      data = server.getNext(5);           % samples within 5 ms are treated as concurrent
      if isnan(data(1)), continue; end
      t = data(1)/1000;
      if isempty(t0), t0 = t; end
      if isnan(s.tPrev), s.tPrev = t; end
      acc = data(1, 2:4)'; gyr = data(1, 5:7)'; mag = data(1, 8:10)'; orient = data(1, 18:21)';
      [s, accOk, magOk] = ekfStep(s, cfg, t, acc, gyr, mag);
      if rem(counter, 10) == 0              % throttle the display
        setOrientation(ownView, conjQuat(s.x));   % viewer expects body->world
        title(ownView, 'OWN', 'FontSize', 16);
        ownView.setAccDist(~accOk); ownView.setMagDist(~magOk);   % flag rejected samples in the viewer
        if ~any(isnan(orient))
          if isempty(googleView), subplot(1, 2, 2); googleView = OrientationView('Google filter', gca); end
          setOrientation(googleView, orient); title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      counter = counter + 1;
      xhat.x(:, end+1) = s.x; xhat.P(:, :, end+1) = s.P; xhat.t(end+1) = t - t0;
      meas.t(end+1) = t - t0; meas.acc(:, end+1) = acc; meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag; meas.orient(:, end+1) = orient;
    end
  catch e
    fprintf('Unsuccessful connecting to client!\nStart streaming from the phone *after* running this function.\n%s\n', e.message);
  end
end
