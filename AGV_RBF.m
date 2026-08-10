function [phi, dphi_dZ] = AGV_RBF(Z, network)
%AGV_RBF Two compact RBF feature maps used by the AGV ICAS controller.
%   network = 'F': Identifier features based on vehicle tracking states.
%   network = 'J': centered Critic/Actor value features based on
%                  [s1; z2_bar], with S_J(0) = 0 and dS_J(0) = 0.
%
%   dphi_dZ(i,j) is the derivative of basis i with respect to Z(j).

if nargin < 2
    network = 'F';
end

Z = Z(:);
if numel(Z) ~= 4
    error('AGV_RBF:InvalidInput', 'Z must contain four elements.');
end

switch upper(network)
    case 'F'
        % Physical tracking-error scales used by the Identifier.
        scales = [0.10; 0.02; 0.50; 0.20];
        center_levels = [1; 1; 1; 1];
        width = 1.0;
        center_value_features = false;
    case 'J'
        % s1 explicitly carries SFPPB proximity information. z2_bar is
        % scaled separately so the heading channel is no longer hidden by
        % the much wider RBFs used for the physical vehicle states.
        scales = [5.0; 5.0; 0.25; 0.25];
        center_levels = [2; 2; 1; 1];
        width = 1.1;
        center_value_features = true;
    otherwise
        error('AGV_RBF:UnknownNetwork', ...
            'network must be either ''F'' or ''J''.');
end

% One origin node and one positive/negative node on each normalized axis.
centers = zeros(4, 9);
for axis_index = 1:4
    centers(axis_index, 2*axis_index) = center_levels(axis_index);
    centers(axis_index, 2*axis_index + 1) = -center_levels(axis_index);
end

normalized_Z = Z./scales;
phi = zeros(9, 1);
dphi_dZ = zeros(9, 4);

for node_index = 1:9
    offset = normalized_Z - centers(:, node_index);
    phi(node_index) = exp(-(offset'*offset)/(2*width^2));
    dphi_dZ(node_index, :) = ...
        (-phi(node_index)/width^2)*(offset./scales)';
end

if center_value_features
    phi_at_origin = zeros(9,1);
    dphi_at_origin = zeros(9,4);
    for node_index = 1:9
        offset = -centers(:,node_index);
        phi_at_origin(node_index) = exp( ...
            -(offset'*offset)/(2*width^2));
        dphi_at_origin(node_index,:) = ...
            (-phi_at_origin(node_index)/width^2)*(offset./scales)';
    end
    phi = phi-phi_at_origin-dphi_at_origin*Z;
    dphi_dZ = dphi_dZ-dphi_at_origin;
end
end
