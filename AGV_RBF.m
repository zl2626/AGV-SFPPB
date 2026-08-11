function S = AGV_RBF(Z,type)
% AGV_RBF  论文中的普通Gaussian RBF基函数
% F网络输入：Z_F=[e_y,e_phi,de_y,de_phi]
% J网络输入：Z_J=[Z_F;s_j]，其中 s_j 是 PI 变换误差
% 公式：S_j(Z)=exp(-(Z-c_j)'(Z-c_j)/a^2)

if nargin < 2
    type = 'F';
end
Z = Z(:);

% ========================== RBF参数 ==========================
N = 7;
a = 1.2;                         % Gaussian宽度

% 物理状态的7个中心
c_F = [ ...
    -1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0;
    -0.3, -0.1, -0.05, 0, 0.05, 0.1, 0.3;
    -0.5, -0.2, 0, 0, 0, 0.2, 0.5;
    -0.2, -0.05, 0, 0, 0, 0.05, 0.2];

% Critic/Actor 的 s_j 中心，仍然是普通 Gaussian，不做原点门控。
c_z = [ ...
    -1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0;
    -1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0];

if upper(type) == 'F'
    if numel(Z) ~= 4
        error('AGV_RBF:Input','F网络输入必须是4维物理状态。');
    end
    c = c_F;
    % 让e_phi和de_phi的量纲与论文中的归一化状态一致。
    scale = [1;0.1;1;0.3];
elseif upper(type) == 'J'
    if numel(Z) ~= 6
        error('AGV_RBF:Input','J网络输入必须是[Z_F;s_j]六维状态。');
    end
    c = [c_F;c_z];
    scale = [1;0.1;1;0.3;1;1];
else
    error('AGV_RBF:Type','type只能是F或J。');
end

Z = Z./scale;
c = c./scale;
S = zeros(N,1);
for j = 1:N
    d = Z-c(:,j);
    S(j) = exp(-(d'*d)/(a^2));
end
