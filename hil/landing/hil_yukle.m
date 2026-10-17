function hil_yukle(csv_yolu)
%% hil_yukle — kaydedilmis HIL CSV'sini calisma alanina geri yukler
%
%  Kart takili olmadan grafikleri yeniden uretmek icin. hil_kos'un
%  yazdigi 20 sutunlu ayrinti CSV'sinden hil_durum / hil_komut
%  yapilarini birebir geri kurar; sonra >> hil_ciz(no) calistirilir.
%
%  Kullanim:
%     >> hil_yukle('AHR_DIKEY_INIS\S-TEK-HIL-ayrinti-no-001.csv')
%     >> hil_ciz(1)
%
%  NOT: sartname CSV'si (adinda bosluk olan, 6 sutunlu) DEGIL, ayrinti
%  CSV'si verilmelidir.

if nargin < 1 || isempty(csv_yolu)
    error('CSV yolu gerekli. Ornek:\n  hil_yukle(''AHR_DIKEY_INIS\\S-TEK-HIL-ayrinti-no-001.csv'')');
end
if ~isfile(csv_yolu)
    alt = fullfile(fileparts(mfilename('fullpath')), csv_yolu);
    if isfile(alt)
        csv_yolu = alt;
    else
        error('CSV bulunamadi: %s', csv_yolu);
    end
end

opt = detectImportOptions(csv_yolu, 'Delimiter', ';');
opt = setvartype(opt, opt.VariableNames, 'double');
opt = setvaropts(opt, opt.VariableNames, 'DecimalSeparator', ',');
T = readtable(csv_yolu, opt);
if width(T) < 20
    error(['Bu %d sutunlu bir dosya. Ayrinti CSV''si 20 sutunludur —\n' ...
           'adinda bosluk OLMAYAN dosyayi ver (sartname CSV''sini degil).'], width(T));
end

t = T.saniye(:);
n = numel(t);

q = euler2quat(T.roll_deg*pi/180, T.pitch_deg*pi/180, T.yaw_deg*pi/180);
D = [T.irtifa_m(:), T.dusey_hiz_m_s(:), q, ...
     T.p_deg_s(:)*pi/180, T.q_deg_s(:)*pi/180, T.r_deg_s(:)*pi/180];

K = [T.itki_N(:), T.lule_pitch_deg(:)*pi/180, T.lule_yaw_deg(:), ...
     T.vana_gerceklesen_deg(:), T.servo_pitch_deg(:), T.servo_yaw_deg(:), ...
     T.itki_istenen_N(:), T.gorev_modu(:), T.paket_gecerli(:), T.seq(:)];

hil_durum.time = t;   hil_durum.signals.values = D;
hil_komut.time = t;   hil_komut.signals.values = K;
assignin('base','hil_durum',hil_durum);
assignin('base','hil_komut',hil_komut);

mod = median(T.gorev_modu);
if mod >= 1.5, ad = 'HOVER'; else, ad = 'DIKEY INIS'; end
fprintf('  yuklendi : %s\n', csv_yolu);
fprintf('  %d ornek, %.2f s, gorev modu %d (%s)\n', n, t(end), round(mod), ad);
fprintf('\n  >> hil_ciz(no)  ile grafikler yeniden uretilir\n\n');
end

function q = euler2quat(roll, pitch, yaw)
    cr = cos(roll/2);  sr = sin(roll/2);
    cp = cos(pitch/2); sp = sin(pitch/2);
    cy = cos(yaw/2);   sy = sin(yaw/2);
    q = [cr.*cp.*cy + sr.*sp.*sy, ...
         sr.*cp.*cy - cr.*sp.*sy, ...
         cr.*sp.*cy + sr.*cp.*sy, ...
         cr.*cp.*sy - sr.*sp.*cy];
end
