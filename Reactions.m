%% Data definition
format bank; clc; clear all;
tau_allowable = 25 * 10^6;
l1 = 41.50e-3;
l2 = 34.67e-3;
l3 = 34.67e-3;
l1_3 = 36.03e-3;
l2_3 = 31.80e-3;

r1 = 250e-3 / 2; % Pulley
r2 = 56.16e-3 / 2; % Pinion
r1_3 = 220.68e-3 / 2; % Gear

Mt_2 = 77.6; % Torque
Mt_3 = 295.1;

% V-belt reactions
theta = (180 - 136)/2;
T1 = 841.76; % Tense side
T2 = 249.54; % Slack side

% Gear reactions (calculated before)
Fcx2 = 1349; % Axial
Fcy2 = 1162; % Radial
Fcz2 = 2893; % Tangential

Fx3 = 1247;
Fy3 = 1074;
Fz3 = 2675;

%% Shaft II
% Forces=[Point A, tense side tension,
%         Point A, slack side tension,
%         Point C, gear forces]
Forces = [Force(0, T1*cosd(theta),  T1*sind(theta), 0, r1, 156), ...
          Force(0, T2*cosd(theta), -T2*sind(theta), 0, r1, 22), ...
          Force(-Fcx2, Fcy2, Fcz2, l1 + l2, r2, 180)];
      
[Fbx, Fby, Fbz, Fdy, Fdz] = shaftBearing(Forces, l1, l1+l2+l3);

Forces = [Forces, ...
          Force(Fbx, Fby, Fbz, l1), ...
          Force(0, Fdy, Fdz, l1+l2+l3)];
       
[x_V, Vy, Vz, x_M, My, Mz] = shaftBending(Forces);

Fbr = double(sqrt(Fby^2 + Fbz^2));
Fdr = double(sqrt(Fdy^2 + Fdz^2));
fprintf('Shaft II: Fr1 = %4.2f, Fr2 = %4.2f, Fa = %4.2f \n\n', ...
    Fbr, Fdr, Fbx);

M = sqrt(My.^2 + Mz.^2);
d = diameter(M, tau_allowable);

%% Shaft III
ForcesIII = [Force(Fx3, -Fy3, Fz3, l1_3, r1_3, 180)];
[Fbx3, Fby3, Fbz3, Fdy3, Fdz3] = shaftBearing(ForcesIII, 0, l1_3 + l2_3);

ForcesIII = [ForcesIII, ...
             Force(Fbx3, Fby3, Fbz3, 0), ...
             Force(0, Fdy3, Fdz3, l1_3 + l2_3)];
[x_V3, Vy3, Vz3, x_M3, My3, Mz3] = shaftBending(ForcesIII);

Fbr3 = double(sqrt(Fby3^2 + Fbz3^2));
Fdr3 = double(sqrt(Fdy3^2 + Fdz3^2));
fprintf('Shaft III: Fr1 = %4.2f, Fr2 = %4.2f, Fa = %4.2f \n\n', ...
    Fbr3, Fdr3, Fbx3);

M3 = sqrt(My3.^2 + Mz3.^2);
d3 = diameter(M3, tau_allowable);

figure(55); hold on;
plot(1000 * x_M3, 1000 *  d3/2, 'b', 'LineWidth', 2);
plot(1000 * x_M3, 1000 * -d3/2, 'b', 'LineWidth', 2);

%% Final strength
sections = { ... d_used, Kf, Mb, Mt
    ["I", 28, 1.8, findBending(26, x_M, M), Mt_2], ...
    ["II", 30, 1.8, findBending(46, x_M, M), Mt_2], ...
    ["III", 32, 1.8, findBending(53.5, x_M, M), Mt_2], ...
    ["IV", 30, 1.8, findBending(106.33, x_M, M), 0], ...
};

n = 3; 
Sy = 330e6; 
Se = 300e6; 
for i = 1 : length(sections)
    Kf = sections{1, i}(1, 3);
    Mb = sections{1, i}(1, 4);
    Tm = sections{1, i}(1, 5);
    min_tors = diameter(Mb, tau_allowable) * 1000;
    min_comb = criticalDiameter(n, Kf, Mb, Tm, Se, Sy) * 1000;
    fprintf(['%s. ', ...
             'Mb: %4.3f, ', ...
             'Min bending: %4.3f, ', ...
             'Min combined: %4.3f, ', ...
             'Selected: %4.3f \n'], ...
       sections{1, i}(1, 1), Mb, min_tors, min_comb, sections{1, i}(1, 2));
end


%% Draw theoretical outline
d_safety = d * 1.07;
SelectedProfile = { ...
    [0, 0], [0, 28], ...
    [55, 28], [55, 30], ...
    [55 + 20, 30], [55 + 20, 35], ...
    [55 + 20 + 7.5, 35], [55 + 20 + 7.5, 32], ...
    [55 + 20 + 7.5 + 52.83, 32], [55 + 20 + 7.5 + 52.83, 30], ...
    [55 + 20 + 7.5 + 52.83 + 15, 30], [55 + 20 + 7.5 + 52.83 + 15, 0] ...
};
x_selected = cellfun(@(vector) vector(1), SelectedProfile) - 29;
y_selected = cellfun(@(vector) vector(2), SelectedProfile);

figure(1); set(gcf, 'color', 'w');
hold on; % Multiply by 1000 to show labels in mm
plot(1000 * x_M, 1000 *  d/2, 'b', 'LineWidth', 2);
plot(1000 * x_M, 1000 * -d/2, 'b', 'LineWidth', 2);
plot(1000 * x_M, 1000 *  d_safety/2, 'g', 'LineWidth', 1.25);
plot(1000 * x_M, 1000 * -d_safety/2, 'g', 'LineWidth', 1.25);
plot(x_selected,  y_selected/2, '-r');
plot(x_selected, -y_selected/2, '-r');
xlim([0, 1000 * (l1 + l2 + l3)]);
xline(0, '-.', 'Pulley');
xline(1000 * l1, '-.', 'Bearing B');
xline(1000 * (l1 + l2), '-.', 'Gear');
xline(1000 * (l1 + l2 + l3), '-.', 'Bearing D', ...
    'LabelHorizontalAlignment', 'left');

grid on; grid minor; box on; axis equal;
title('Theoretical outline of shaft (minimal diameters)');
xlabel('Shaft length [mm]');
ylabel('Shaft width [mm]');
legend('Theoretical diameters', '', ...
       'Theoretical diameters with 7% error margin');%, '', ...
       %'Selected diameters');

%% Draw shear forces and moments
figure(2); set(gcf, 'color', 'w');
subplot(2, 1, 1); plot(x_V * 1000, Vy, 'LineWidth', 1.5);
grid on; grid minor; box on;
title('Shear force in the XY plane');
xlabel('x [mm]'); 
ylabel('V_y [N]'); yline(0, '--');
xlim([0, (l1 + l2 + l3) * 1000]);

subplot(2, 1, 2); plot(x_M * 1000, My, 'LineWidth', 1.5);
grid on; grid minor; box on; axis tight;
title('Bending moments in the XY plane');
xlabel('x [mm]');
ylabel('M_y [Nm]'); yline(0, '--');

figure(3); set(gcf, 'color', 'w');
subplot(2, 1, 1); plot(x_V * 1000, Vz, 'LineWidth', 1.5);
grid on; grid minor; box on;
title('Shear force in the XZ plane');
xlabel('x [mm]');
ylabel('V_z [N]'); yline(0, '--');
xlim([0, (l1 + l2 + l3) * 1000]);

subplot(2, 1, 2); plot(x_M * 1000, Mz, 'LineWidth', 1.5);
grid on; grid minor; box on; axis tight;
title('Bending moments in the XZ plane');
xlabel('x [mm]');
ylabel('M_z [Nm]'); yline(0, '--');

figure(4); set(gcf, 'color', 'w');
plot(x_M, M, 'LineWidth', 1.5);
grid on; grid minor; box on; axis tight;
title('Equivalent bending moments');
xlabel('x [mm]');
ylabel('M [Nm]'); yline(0, '--');

%% Bearing calculation, F1 - bearing 1, F2 - bearing 2
function [F1x, F1y, F1z, F2y, F2z] = shaftBearing(Forces, l_F1, l_F2)
    syms F1x F1y F1z F2y F2z
    Forces = [Forces, ...
        Force(F1x, F1y, F1z, l_F1), Force(0, F2y, F2z, l_F2)];
    
    equations = [ ...
        sum([Forces.X]) == 0, ... % ΣFx
        sum([Forces.Y]) == 0, ... % ΣFy
        sum([Forces.Z]) == 0, ... % ΣFz
        sum([-[Forces.Z] .* [Forces.L], ...
              [Forces.X] .* [Forces.R] .* cosd([Forces.Theta]) ...
        ]) == 0, ... % ΣMy
        sum([[Forces.Y] .* [Forces.L], ...
             [Forces.X] .*[Forces.R] .* sind([Forces.Theta]) ...
        ]) == 0, ... % ΣMz
    ];

    [F1x, F1y, F1z, F2y, F2z] = solve(equations, [F1x F1y F1z F2y F2z]);
end

%% Bending calculation
function [x_V, Vy, Vz, x_M, My, Mz] = shaftBending(Forces)
    syms x;
    [n, lengths] = getSections(Forces);
    len = sumPairs(repelem(x, n), -lengths);
    
     x_V = zeros(1, 2*n);
     for i = 1 : n
         x_V(2*i - 1) = lengths(i);
         x_V(2*i) = lengths(i);
     end
    
    Vy = sym(zeros(1, 2*n));
    for i = 1 : n - 1
        value = sum([Forces(isInRange(Forces, i, lengths)).Y]) + Vy(2*i-1);
        Vy(2*i) = value;
        Vy(2*i + 1) = value;
    end
    
    Vz = sym(zeros(1, 2*n));
    for i = 1 : n - 1
        value = sum(-[Forces(isInRange(Forces, i, lengths)).Z]) + Vz(2*i-1);
        Vz(2*i) = value;
        Vz(2*i + 1) = value;
    end
    
    My_formula = sym(zeros(1, n - 1));
    for i = 1 : n - 1
        F = Forces(isInRange(Forces, i, lengths));
        My_formula(i) = sum([ ...
            [F.Y] .* repelem(len(i), length(F)), ...
            [F.X] .* [F.R] .* sind([F.Theta]), ...
        ]);
        if (i - 1 > 0) % Add previous moments
            My_formula(i) = My_formula(i) + My_formula(i - 1);
        end
    end
    
    Mz_formula = sym(zeros(1, n - 1));
    for i = 1 : n - 1
        F = Forces(isInRange(Forces, i, lengths));
        Mz_formula(i) = sum([ ...
           -[F.Z] .* repelem(len(i), length(F)), ...
           -[F.X] .* [F.R] .* cosd([F.Theta]), ...
        ]);
        if (i - 1 > 0) % Add previous moments
            Mz_formula(i) = Mz_formula(i) + Mz_formula(i - 1);
        end
    end
    
%     vpa(My_formula, 5)
%     figure(155); hold on;
%     Ty_formula = diff(My_formula);
%     Ty = 
%     for i = 1 : n - 1
%         temp = subs(Ty_formula(i), x, [x_V(2*i - 1), x_V(2*i)]);
%         plot(x_V, temp);
%     end
%     
%     vpa(Mz_formula, 5)
%     figure(156); hold on;
%     Tz_formula = diff(Mz_formula);
%     
%     for i = 1 : n - 1
%         temp = subs(Tz_formula(i), x, [x_V(2*i - 1), x_V(2*i)]);
%         plot(x_V, temp);
%     end
    
    x_M = []; My = []; Mz = [];
    for i = 1 : n - 1 % Interpolate moment values for plots
        x_section = lengths(i) : 10e-5 : lengths(i + 1);
        x_M = [x_M x_section];
        My = [My subs(My_formula(i), x, x_section)];
        Mz = [Mz subs(Mz_formula(i), x, x_section)];
    end
end

%% Auxilary functions
function Vector = sum(Elements)
    Vector = Elements(1);
    for i = 2 : length(Elements)
      Vector = Vector + Elements(i);
    end
end

function Vector = sumPairs(Elements_A, Elements_B)
    if length(Elements_A) ~= length(Elements_B)
        error('Length of two vectors is not equal')
    end

    syms Vector 
    Vector(1) = Elements_A(1);
    for i = 2 : length(Elements_A) % Omit first element
      Vector(i) = Elements_A(i) + Elements_B(i);
    end
    Vector = Vector(1 : end - 1);
end

function [n, lengths] = getSections(Forces)   
    lengths = [];
    for i = 1 : length(Forces)
        already_exists = false;
        for j = 1 : length(lengths)
            if (Forces(i).L == lengths(j))
                already_exists = true;
                break;
            end
        end
        if (~already_exists)
            lengths = [lengths Forces(i).L];
        end
    end
    lengths = sort(lengths);
    if (isempty(lengths)) 
        n = 0;
    else
        n = length(lengths);
    end
end

function Array = getLengthForceArray(Forces) 
    Array = cell(1, length(Forces));
    for i = 1 : length(Forces)
       Array{i} = [Forces(i).L, Forces(i)];
    end
    Array = sortrows(Array, 1);
end

function Result = isInRange(Forces, section, lengths)
    Result = false(1, length(Forces));
    for i = 1 : length(Forces)
        if (Forces(i).L >= lengths(section) && ...
                Forces(i).L < lengths(section + 1))
        
            Result(i) = true;
        end
    end
end

function Mb = findBending(x, x_M, M)
    x = x / 1000; %Find the closest value and index to x in x_M
    [~, index] = min(abs(x_M - x));
    Mb = M(index);
end

%% Diameter calculation
function d = diameter(T, tau_allowable)
    d = ((16*T) ./ (pi*tau_allowable)).^(1/3);
end

function d = criticalDiameter(n, Kf, Mb, Tm, Se, Sy)
    d = (32*n/pi * (((Kf*Mb/Se)^2 + 3/4 * (Tm/Sy)^2))^(1/2))^(1/3);
end