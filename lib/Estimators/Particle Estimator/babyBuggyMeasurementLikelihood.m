function likelihood = babyBuggyMeasurementLikelihood(predictParticles,measurement,varargin)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

numberOfMeasurements = 3; % Expected number of measurements

% % Validate the measurement
% validateattributes(measurement, {'double'}, {'vector', 'numel', numberOfMeasurements}, ...
%     'vdpMeasurementLikelihoodFcn', 'measurement');

% Assume that measurements are subject to Gaussian distributed noise with
% variance 0.016
% Specify noise as covariance matrix
measurementNoise = 0.016 * eye(numberOfMeasurements);
  
% The measurement contains the first state variable. Get the first state of
% all particles
predictedMeasurement = predictParticles(1:3, :);


% Calculate error between predicted and actual measurement
measurementError = bsxfun(@minus, predictedMeasurement, measurement);

% Use measurement noise and take inner product
measurementErrorProd = dot(measurementError, measurementNoise \ measurementError, 1);

% Convert error norms into likelihood measure. 
% Evaluate the PDF of the multivariate normal distribution. A measurement
% error of 0 results in the highest possible likelihood.
likelihood = 1/sqrt((2*pi).^numberOfMeasurements * det(measurementNoise)) * exp(-0.5 * measurementErrorProd);

end

