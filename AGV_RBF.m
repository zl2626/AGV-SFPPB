% function phi = AGV_RBF(Z)
%     % Z: [e_y, e_phi, de_y, de_phi]
% 
%     % 基于误差动态调整中心点
%     dynamic_centers = [
%         -1.0, -0.5, -0.8, -0.3;
%         -0.5, -0.2, -0.3, -0.1; 
%          0.5,  0.2,  0.3,  0.1;
%          1.0,  0.5,  0.8,  0.3]';
% 
%     % 自适应宽度
%     error_norm = norm(Z(1:2));  % e_y和e_phi的范数
%     base_width = 1.5;
%     adaptive_widths = base_width * (1 + 0.5 * tanh(error_norm));
% 
%     phi = zeros(4,1);
%     for i = 1:4
%         phi(i) = exp(-norm(Z - dynamic_centers(:,i))^2 / ...
%                     (2 * adaptive_widths^2));
%     end
% end

function [phi] = AGV_RBF(Z)
    % Z: [e_y, e_phi, de_y, de_phi]
    
    % 使用更合理的中心点布局
    centers = [
        -1.0, -0.5, -0.2,  0.0,  0.2,  0.5,  1.0;  % e_y
        -0.3, -0.1, -0.05, 0.0,  0.05, 0.1,  0.3;  % e_phi  
        -0.5, -0.2,  0.0,  0.0,  0.0,  0.2,  0.5;  % de_y
        -0.2, -0.05, 0.0,  0.0,  0.0,  0.05, 0.2   % de_phi
    ];
    
    total_nodes = 7;
    base_width = 0.8;  % 减小宽度，提高局部性
    
    phi = zeros(total_nodes, 1);
    
    for i = 1:total_nodes
        distance = norm(Z - centers(:,i));
        % 添加距离限制，避免数值问题
        if distance > 5.0
            phi(i) = 0;
        else
            phi(i) = exp(-distance^2 / (2 * base_width^2));
        end
    end
    
    % 确保phi不会过大
    phi_norm = norm(phi);
    if phi_norm > 10.0
        phi = phi * 10.0 / phi_norm;
    end
end