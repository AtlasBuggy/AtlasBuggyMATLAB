function predictedParticles = babyBuggyStateTransition(particles,varargin)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    dt = varargin{1};
    update_type = varargin{2};
    update_val = varargin{3};
    
    if(strcmp(update_type, 'encoder'))
        predictedParticles = updateEncoder(particles, update_val, dt);
    elseif(strcmp(update_type, 'imu'))
        predictedParticles = updateIMU(particles, update_val, dt);
    else
        predictedParticles = particles;
    end
end

function predictedParticles = updateEncoder(particles, update_val, dt)
    [numberOfStates, numberOfParticles] = size(particles);
    
    a = 0.5;
    b = 1;

    v0 = update_val;
    
    % Euler integration of continuous-time dynamics x'=f(x) with sample time dt
    for kk=1:numberOfParticles
        x = particles(:,kk);
        
        theta = x(3);
%         gamma = x(4);
%         alpha = atan2(a*tan(gamma), b);
        alpha = 0;
        
        x_dot = zeros(9,1);
        x_dot(1) = v0 * cos(alpha + theta) / cos(alpha);
        x_dot(2) = v0 * sin(alpha + theta) / cos(alpha);
        x_dot(3) = x(7);
        
        particles(:,kk) = x + x_dot*dt;
    end
    
    processNoise = 0.025*eye(numberOfStates);
    predictedParticles = particles + processNoise * randn(size(particles));
end

function predictedParticles = updateIMU(particles, update_val, ~)
    [numberOfStates, numberOfParticles] = size(particles);
    
    % Euler integration of continuous-time dynamics x'=f(x) with sample time dt
    for kk=1:numberOfParticles
        x = particles(:,kk);
        
        x(7) = update_val;
        
        particles(:,kk) = x;
    end
    
    processNoise = 0.025*eye(numberOfStates);
    predictedParticles = particles + processNoise * randn(size(particles));
end