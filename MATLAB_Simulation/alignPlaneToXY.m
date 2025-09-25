function R = alignPlaneToXY(p1, p2, p3)
%ALIGNPLANETOXY  Compute rotation to map the plane through p1,p2,p3 to z=0.
%
%   R = alignPlaneToXY(p1,p2,p3) returns a 3×3 rotation matrix R such that
%   if you center your points at p1:
%
%       P = [p1, p2, p3];
%       P_centered = P - p1;      % subtract p1 from all
%       P_rot = R * P_centered;   % rotate
%
%   then P_rot(:,2) and P_rot(:,3) will have zero z-component (i.e. lie in z=0).

    % form two in-plane vectors
    v1 = p2 - p1;
    v2 = p3 - p1;

    % plane normal (unit)
    n = cross(v1, v2);
    n = n / norm(n);

    % desired normal (z-axis)
    k = [0; 0; 1];

    % check for degenerate cases: already aligned or opposite
    if abs(dot(n,k) - 1) < 1e-8
        % plane already horizontal
        R = eye(3);
        return
    elseif abs(dot(n,k) + 1) < 1e-8
        % plane upside-down: 180° about any axis in XY plane, e.g. x
        R = [1 0  0;
             0 -1 0;
             0 0 -1];
        return
    end

    % rotation axis (unit) and angle
    u = cross(n, k);
    u = u / norm(u);
    theta = acos(dot(n, k));

    % skew-symmetric for u
    K = [   0   -u(3)  u(2);
          u(3)    0   -u(1);
         -u(2)  u(1)    0   ];

    % Rodrigues’ formula
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
end