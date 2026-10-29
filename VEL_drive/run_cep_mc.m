function run_cep_mc(N)
    if nargin < 1, N = 40; end
    impacts = zeros(N,2);
    nominal = [0;0;-4.5;449.5;0;-404.8;0.93358;0;0.35837;0;1131;0;0;0;1131];
    
    % Seed workspace variable for the Integrator block
    assignin('base','IC0',nominal);

    for i = 1:N
        fprintf('Run %d/%d ... ', i, N);
        % Launch dispersions (realistic 155 mm)
        dV0 = 5.0*randn();                     % muzzle velocity ±5 m/s (1σ)
        dQE = 0.05*pi/180*randn();             % quadrant elevation ±0.05°
        dAz = 0.05*pi/180*randn();             % azimuth ±0.05°
        V0 = 605 + dV0;
        QE = 42*pi/180 + dQE;
        Az = dAz;
        Vn = V0*cos(QE)*cos(Az);
        Ve = V0*cos(QE)*sin(Az);
        Vd = -V0*sin(QE);
        
        % Quaternion composition for perturbed angles
        qp = [cos(QE/2); 0; sin(QE/2); 0];
        qy = [cos(Az/2); 0; 0; sin(Az/2)];
        q  = [qy(1)*qp(1) - dot(qy(2:4), qp(2:4));
              qy(1)*qp(2:4) + qp(1)*qy(2:4) + cross(qy(2:4), qp(2:4))];
              
        ic = [0; 0; -4.5; Vn; Ve; Vd; q(1); q(2); q(3); q(4); 1131; 0; 0; 0; 1131];
        
        % Assign perturbed IC to workspace
        assignin('base','IC0',ic);
        
        % Run Simulink
        out = sim('proof_of_concept');
        X = out.simout.Data;
        h = -X(:,3);
        k = find(h >= 0, 1, 'last');
        if isempty(k), k = size(X,1); end
        impacts(i, :) = X(k, 1:2);
        fprintf('impact (%.1f, %.1f) m\n', impacts(i,1), impacts(i,2));
    end

    % Restore nominal IC
    assignin('base','IC0',nominal);

    % Calculate CEP
    centroid = mean(impacts, 1);
    r = sqrt(sum((impacts - centroid).^2, 2));
    r_sorted = sort(r);
    CEP50 = r_sorted(max(1, round(0.50*N)));
    CEP90 = r_sorted(max(1, round(0.90*N)));
    
    fprintf('\n=== Results ===\n');
    fprintf('Centroid : (%.1f, %.1f) m\n', centroid(1), centroid(2));
    fprintf('CEP50    : %.1f m\n', CEP50);
    fprintf('CEP90    : %.1f m\n', CEP90);

    % Plot Dispersion
    figure('Color','w','Position',[100 100 700 700]);
    scatter(impacts(:,1), impacts(:,2), 50, 'b', 'filled'); hold on; grid on;
    th = linspace(0, 2*pi, 200);
    plot(centroid(1)+CEP50*cos(th), centroid(2)+CEP50*sin(th), 'r-', 'LineWidth', 2);
    plot(centroid(1)+CEP90*cos(th), centroid(2)+CEP90*sin(th), 'r--', 'LineWidth', 2);
    plot(centroid(1), centroid(2), 'k+', 'MarkerSize', 15, 'LineWidth', 2);
    axis equal;
    xlabel('North (m)'); ylabel('East (m)');
    title(sprintf('Impact Dispersion (N=%d) | CEP50 = %.1f m, CEP90 = %.1f m', ...
                  N, CEP50, CEP90));
    legend({'Impacts', sprintf('CEP50 = %.1f m', CEP50), ...
                    sprintf('CEP90 = %.1f m', CEP90), 'Centroid'}, 'Location','best');
end