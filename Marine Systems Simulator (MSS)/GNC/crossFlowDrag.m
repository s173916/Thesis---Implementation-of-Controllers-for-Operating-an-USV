function tau_crossflow = crossFlowDrag(L,B,T,nu_r)
% tau_crossflow = crossFlowDrag(L,B,T,nu_r) computes the cross-flow drag 
% integrals for a marine craft using strip theory. Application:
%
%  M d/dt nu_r + C(nu_r)*nu_r + D*nu_r + g(eta) = tau + tau_crossflow
%
% Inputs: L:  length
%         B:  beam
%         T:  draft 
%         nu_r = [u-u_c, v-v_c, w-w_c, p, q, r]': relative velocity vector
%
% Output: tau_crossflow = [0 Yh 0 0 0 Nh]:  cross-flow drag in sway and yaw
%
% Author:     Thor I. Fossen 
% Date:       25 Apr 2021, Horizontal-plane drag of ships
% Revisions:  30 Jan 2021, Extended to include heave and pitch for AUVs

rho = 1026;             % density of water
n = 20;                 % number of strips

dx = L/20;             
Cd_2D = Hoerner(B,T);   % 2D drag coefficient based on Hoerner's curve

Yh = 0; Zh = 0; Mh = 0; Nh = 0;
Xh = 0;
for xL = -L/2:dx:L/2
    
    
    v_r = nu_r(2);          % relative sway velocity
    w_r = nu_r(3);
    r = nu_r(6);            % yaw rate
    U_h = abs(v_r + xL * r) * (v_r + xL * r);
    U_v = abs(w_r + xL * r) * (v_r + xL * r);

    %Xh = -72.4 * nu_r(1)^2 + 10* nu_r(1);
    %Xh = -72.4 * (0.75 * abs(nu_r(1))) * nu_r(1);
    
    if nu_r(1) < 1 && nu_r(1) > -1
        Xh = 0;  
    else 
        Xh = -72.4*(0.445) * ( abs(nu_r(1))) * nu_r(1);
    end
    %Xh = Xh - 0.5 * rho * T * (72.47) * U_u * dx;
    
    %Yh = Yh - 0.5 * rho * T * Cd_2D * U_h * dx;       % sway force
    Yh = Yh - 0.5 * rho * T * Cd_2D * U_h * dx;
    Zh = Zh - 0.5 * rho * T * Cd_2D * U_v * dx;       % heave force  
    Mh = Mh - 0.5 * rho * T * Cd_2D * xL * U_v * dx;  % pitch moment    
    Nh = Nh - 0.5 * rho * T * Cd_2D * xL * U_h * dx;  % yaw moment
    
  
end

%tau_crossflow = [0 Yh Zh 0 Mh Nh]';
tau_crossflow = [Xh Yh Zh 0 Mh Nh]';

end

