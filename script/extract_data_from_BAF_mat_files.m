% Dopo aver caricato in workspace la struttura sub9_trial2_task4:
% >> load('sub9_trial2_task4.mat')
save_imu_full_csv(robot_logger_device, 'ExtractedNodeRawDataCSV', 0.016, 3:12);



function save_imu_full_csv(S, outdir, dt, node_range)
% save_imu_full_csv  Estrae orientation (quat), angVel (gyro), linAcc e magnetometro e salva CSV per nodo.
%
% USO:
%   save_imu_full_csv(sub9_trial2_task4, 'out_csv', 0.016, 3:12)
%
% INPUT:
%   S          : struttura radice (es. sub9_trial2_task4)
%   outdir     : cartella di output (creata se non esiste)
%   dt         : sampling period in secondi (es. 0.016 per 60 Hz)
%   node_range : vettore dei nodi da processare (default 3:12)
%
% OUTPUT:
%   Crea file CSV per ogni nodo:
%     time_s, imu_id, qw,qx,qy,qz, gyro_x_dps,gyro_y_dps,gyro_z_dps, acc_x_ms2,acc_y_ms2,acc_z_ms2, mag_x_uT,mag_y_uT,mag_z_uT
%
% NOTE:
%   - Esegue squeeze e trasposizione se i dati arrivano come (3 x n) o (4 x n) per i quaternioni.
%   - Allinea le lunghezze dei segnali tronacando al minimo comune disponibile tra quelli presenti.
%   - Se un segnale manca, la funzione salva comunque con colonne riempite di NaN.
%   - Adatta eventuali conversioni unità (gyro rad/s -> deg/s, acc g -> m/s^2, mag in G -> uT) nei punti indicati.

if nargin < 4 || isempty(node_range), node_range = 3:12; end
if nargin < 3 || isempty(dt), dt = 0.016; end
if nargin < 2 || isempty(outdir), outdir = 'out_csv'; end

if ~isstruct(S)
    error('Il primo argomento deve essere la struttura radice (es. sub9_trial2_task4).');
end

if ~exist(outdir, 'dir')
    mkdir(outdir);
end

for n = node_range
    nodeName = sprintf('node%d', n);
    if ~isfield(S, nodeName)
        warning('Campo mancante: %s. Salto.', nodeName);
        continue;
    end
    node = S.(nodeName);

    % --- Letture robuste con nomi alternativi ---
    % Magnetometro
    M = try_get_matrix(node, {'magnetometer'}, 'data', 3);   % -> n x 3 o []
    % Accelerometro lineare
    A = try_get_matrix(node, {'linAcc','acc','accelerometer','accel'}, 'data', 3);
    % Gyro / angVel (ATTESO in deg/s; vedi conversione sotto se in rad/s)
    G = try_get_matrix(node, {'angVel','gyro','gyroscope'}, 'data', 3);
    % Quaternioni (ATTESO in ordine [w x y z]; vedi riordinamento se serve)
    Q = try_get_matrix(node, {'orientation','quaternion','quat'}, 'data', 4);

    % --- Allineamento lunghezze (tronca al minimo tra quelli NON vuoti) ---
    lens = [];
    labels = {};
    if ~isempty(Q), lens(end+1) = size(Q,1); labels{end+1}='Q'; end
    if ~isempty(G), lens(end+1) = size(G,1); labels{end+1}='G'; end
    if ~isempty(A), lens(end+1) = size(A,1); labels{end+1}='A'; end
    if ~isempty(M), lens(end+1) = size(M,1); labels{end+1}='M'; end

    if isempty(lens)
        warning('%s: nessun segnale trovato (orientation/angVel/linAcc/magnetometer). Salto.', nodeName);
        continue;
    end

    nMin = min(lens);
    if numel(unique(lens)) > 1
        % Crea messaggio leggibile tipo "Q=500, G=498, A=500, M=500"
        pairs = cellfun(@(lab,len) sprintf('%s=%d', lab, len), labels, num2cell(lens), 'UniformOutput', false);
        
        % Se hai una MATLAB recente:
        len_msg = strjoin(pairs, ', ');
        
        % Se la tua versione non avesse strjoin, usa:
        % len_msg = sprintf('%s, ', pairs{:});
        % if ~isempty(len_msg), len_msg(end-1:end) = []; end
        
        warning('%s: lunghezze diverse (%s) -> tronco a %d campioni.', nodeName, len_msg, nMin);

    end

    if ~isempty(Q), Q = Q(1:nMin, :); end
    if ~isempty(G), G = G(1:nMin, :); end
    if ~isempty(A), A = A(1:nMin, :); end
    if ~isempty(M), M = M(1:nMin, :); end

    % --- Tempo ---
    t = (0:nMin-1).' * dt;

    % --- Conversioni unità (ATTIVA se necessario) ---
    % Se G è in rad/s e vuoi deg/s:
    if ~isempty(G), G = rad2deg(G); end
    %
    % Se A è in g e vuoi m/s^2:
    % if ~isempty(A), A = A * 9.80665; end
    %
    % Se M è in gauss (G) e vuoi microTesla (uT): 1 G = 1e5 uT
    % if ~isempty(M), M = M * 1e5; end

    % --- Quaternioni: assicurati dell'ordine (w,x,y,z) ---
    % Se i tuoi quaternioni sono [x y z w], riordina:
    % Q = Q(:, [4 1 2 3]);

    % --- Prepara colonne con NaN se mancanti ---
    nanQ = NaN(nMin,4);
    nanG = NaN(nMin,3);
    nanA = NaN(nMin,3);
    nanM = NaN(nMin,3);
    if isempty(Q), Q = nanQ; end
    if isempty(G), G = nanG; end
    if isempty(A), A = nanA; end
    if isempty(M), M = nanM; end

    % --- Tabella finale (ordine colonne stabile) ---
    T = table();
    T.time_s      = t;
    T.imu_id      = repmat(string(nodeName), nMin, 1);
    T.qw          = Q(:,1);
    T.qx          = Q(:,2);
    T.qy          = Q(:,3);
    T.qz          = Q(:,4);
    T.gyro_x_dps  = G(:,1);
    T.gyro_y_dps  = G(:,2);
    T.gyro_z_dps  = G(:,3);
    T.acc_x_ms2   = A(:,1);
    T.acc_y_ms2   = A(:,2);
    T.acc_z_ms2   = A(:,3);
    T.mag_x_uT    = M(:,1);
    T.mag_y_uT    = M(:,2);
    T.mag_z_uT    = M(:,3);

    % --- Salvataggio con gestione suffissi se file esiste ---
    baseOutFile = fullfile(outdir, sprintf('%s.csv', nodeName));
    outFile = baseOutFile;
    
    % Se il file esiste già, aggiungi suffisso progressivo
    if exist(outFile, 'file')
        suffix = 1;
        while true
            outFile = fullfile(outdir, sprintf('%s_%d.csv', nodeName, suffix));
            if ~exist(outFile, 'file')
                break;
            end
            suffix = suffix + 1;
        end
    end
    
    try
        writetable(T, outFile);
        fprintf('Salvato: %s (%d campioni)\n', outFile, nMin);
    catch ME
        warning('Errore salvataggio %s: %s', outFile, ME.message);
    end
end
end

% =======================
% Helper robusto di lettura
% =======================
function M = try_get_matrix(node, fieldNamesCell, leafName, expectedCols)
% Cerca un campo tra quelli elencati in fieldNamesCell, poi il sotto-campo leafName (tipicamente 'data'),
% fa squeeze e riporta una matrice n x expectedCols (trasponendo se necessario).
%
% Se non trova nulla, restituisce [].
%
% expectedCols: 3 per vettori (mag/acc/gyro), 4 per quaternioni
M = [];

for k = 1:numel(fieldNamesCell)
    f = fieldNamesCell{k};
    if isfield(node, f)
        leaf = node.(f);
        if isstruct(leaf) && isfield(leaf, leafName)
            raw = leaf.(leafName);
        elseif ~isstruct(leaf)
            raw = leaf; % qualcuno salva direttamente la matrice nel campo principale
        else
            continue;
        end

        raw = squeeze(raw);
        sz = size(raw);
        if numel(sz) == 2
            % attesi [expectedCols x n] oppure [n x expectedCols]
            if sz(1) == expectedCols
                M = raw.'; % n x expectedCols
            elseif sz(2) == expectedCols
                M = raw;   % n x expectedCols
            else
                warning('Dimensioni inattese per %s.%s (%s): atteso *x%d. Salto.', f, leafName, mat2str(sz), expectedCols);
                M = [];
            end
        else
            warning('Dimensioni non 2D per %s.%s (%s). Salto.', f, leafName, mat2str(sz));
            M = [];
        end

        if ~isempty(M)
            return;
        end
    end
end
% Se arriva qui, non ha trovato nulla.
end
