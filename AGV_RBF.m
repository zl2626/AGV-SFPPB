function [S,dS] = AGV_RBF(Z,type)
% AGV_RBF  论文中的高斯RBF基函数
% type='F'：Identifier输入物理状态Z_F=[e_y,e_phi,de_y,de_phi]
% type='J'：Critic/Actor输入Z_J=[Z_F,z_j]
% 返回S和对最后两个变换误差的偏导dS。

if nargin < 2
    type = 'F';
end
Z = Z(:);

% =========================== RBF参数 ==================================
N = 7;                         % 节点数
c = 1.5;                       % 节点中心距离
b = 1.1;                       % 高斯宽度

if upper(type) == 'F'
    if numel(Z) ~= 4
        error('AGV_RBF:Input','F网络输入必须是4维物理状态');
    end
    l = [1;0.1;1;0.3];         % 各物理量的尺度
    d = [ 1,  1,  1;
          1, -1,  1;
          1,  1, -1;
          1, -1, -1];
    centered = 0;
else
    if numel(Z) ~= 6
        error('AGV_RBF:Input','J网络输入必须是[Z_F;z_j]六维状态');
    end
    l = [1;0.1;1;0.3;1;1];
    d = [ 1,  1,  1;
          1, -1,  1;
          1,  1, -1;
          1, -1, -1;
         -1,  1,  1;
         -1, -1,  1];
    centered = 1;
end
% 三个正方向、三个负方向和原点，共N个节点。
d = d./sqrt(sum(d.^2,1));
center = [zeros(numel(Z),1),c*d,-c*d];

q = Z./l;
S = zeros(N,1);
dS = zeros(N,numel(Z));
for i = 1:N
    qi = q-center(:,i);
    S(i) = exp(-(qi'*qi)/(2*b^2));
    dS(i,:) = -(S(i)/b^2)*(qi./l)';
end

if centered
    % 让J特征在原点满足S(0)=0、dS(0)=0，避免零误差时产生方向盘偏置。
    q0 = -center;
    S0 = exp(-sum(q0.^2,1)'/(2*b^2));
    gate = 1-exp(-0.5*(q'*q));
    dgate = exp(-0.5*(q'*q))*(q./l)';
    S_old = S;
    S = gate*(S-S0);
    dS = gate*dS+(S_old-S0)*dgate;
end
end
