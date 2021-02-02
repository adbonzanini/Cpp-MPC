%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script creates the spase matrix E to enforce non-anticipativity 
% in multi-stage MPC.
%
% Caution: only works for nu = 1 for the time being
%
% Written by: Angelo D. Bonzanini
% Date: January 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

% User inputs
nu = 1; % Do not change yet
Np = 5;
nScenarios = 4; % nScenarios = Nrobust*nScenariosPerNode;
nCommonNodes=[2, 1, 2]; %scenarios (1&2, 2&3, 3&4)

% Define dimensions
EfullRows = nu*sum(nCommonNodes);
EfullCols = nu*Np*nScenarios;

% Initialize cells and matrices to store the data
Epart = cell(nScenarios);
Efull = zeros(EfullRows, EfullCols);

% Create the parts of the E matrix
for j = 1:length(nCommonNodes)
    Epart{j, j+1} = [eye(nCommonNodes(j)), zeros(nCommonNodes(j),Np-nCommonNodes(j))];
end

% Get the start indices for the substitutions
idxRow = 1;
for j=1:length(nCommonNodes)-1
    idxRow = [idxRow,idxRow(end)+nCommonNodes(j)];
end
idxCol = 1:Np:Np*(nScenarios-1);

% Fill the rest of the matrix with the parts of E calculated above
for k = 1:nScenarios-1
    
    i = idxRow(k);
    j = idxCol(k);
    
    nRows = size(Epart{k,k+1},1);
    nCols = size(Epart{k,k+1},2);
    
    Efull(i:i+nRows-1,j:j+2*nCols-1) = [Epart{k,k+1}, -Epart{k,k+1}]; 
end

%% Extract matrices E_1, ..., E_nScenarios
E = cell(nScenarios,1);
for j=1:nScenarios
    nStart = 1+(j-1)*Np;
    nEnd = j*Np;
    E{j} = Efull(:,nStart:nEnd);
end

%% Test to see if the non-anticipativity constraints are correct
u = sym('u', [Np,nScenarios]);

sum = zeros(EfullRows,1);
for j =1:nScenarios
    sum = sum+E{j}*u(:,j);
end

disp(sum)


