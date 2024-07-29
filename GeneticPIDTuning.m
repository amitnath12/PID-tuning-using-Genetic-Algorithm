clc;
close all;
clear all;
% Objective function to minimize (e.g., Integral of Absolute Error)
function J = pidObjective(params, sys)
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    pidController = pid(Kp, Ki, Kd);
    closed_loop_sys = feedback(pidController * sys, 1);
    [y , t] = step(closed_loop_sys);
    error = 1 - y; % Assume setpoint is 1
    J = sum(abs(error)); % Integral of Absolute Error
end

% Genetic Algorithm for PID Tuning
populationSize = 50; % Number of chromosomes
numGenes = 3; % Number of genes in each chromosome (Kp, Ki, Kd)
numGenerations = 100; % Number of generations
crossoverRate = 0.8; % Probability of crossover
mutationRate = 0.01; % Probability of mutation
a = 0.1; % lower limit of a gene
b = 10 ; %upper limit of a gene

% Initialize population
population = rand(populationSize, numGenes);
sys = tf(1, [1 1 4]); % Example system
closed_loop = feedback(sys, 1); %closed loop without PID
figure;
step(closed_loop);
title('Closed-Loop Step Response without PID');

for generation = 1:numGenerations
    % Evaluate fitness
    fitness = zeros(populationSize, 1);
    for i = 1:populationSize
        fitness(i) = pidObjective(population(i, :), sys);
    end
    
    % Selection (roulette wheel selection)
    totalFitness = sum(fitness);
    selectionProb = fitness / totalFitness;
    selectedIndices = randsample(1:populationSize, populationSize, true, selectionProb);
    selectedPopulation = population(selectedIndices, :);
    
    % Crossover
    for i = 1:2:populationSize
        if rand < crossoverRate
            crossoverPoint = randi([1, numGenes-1]);
            offspring1 = [selectedPopulation(i, 1:crossoverPoint), selectedPopulation(i+1, crossoverPoint+1:end)];
            offspring2 = [selectedPopulation(i+1, 1:crossoverPoint), selectedPopulation(i, crossoverPoint+1:end)];
            selectedPopulation(i, :) = offspring1;
            selectedPopulation(i+1, :) = offspring2;
        end
    end
    
     % Mutation (can be ignored also)
    for i = 1:populationSize
        for j = 1:numGenes
            if rand < mutationRate
                selectedPopulation(i, j) = a + (b-a) * rand;
            end
        end
    end

    % Replace old population with new population
    population = selectedPopulation;
    
    % Check for convergence (optional)
    if max(fitness) > 0.95 %fitness above which solution will be acceptable
        break;
    end
end

% Find the best solution
[~, bestIndex] = min(fitness);
bestParams = population(bestIndex, :);

% Display optimal parameters
Kp_opt = bestParams(1);
Ki_opt = bestParams(2);
Kd_opt = bestParams(3);
fprintf('Optimized PID parameters:\n');
fprintf('Kp = %.2f\n', Kp_opt);
fprintf('Ki = %.2f\n', Ki_opt);
fprintf('Kd = %.2f\n', Kd_opt);

% Simulate the optimized PID controller
pid_opt = pid(Kp_opt, Ki_opt, Kd_opt);
closed_loop_opt = feedback(pid_opt * sys, 1);
figure;
step(closed_loop_opt);
title('Closed-Loop Step Response with Optimized PID');
grid on;
