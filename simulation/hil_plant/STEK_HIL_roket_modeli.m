function STEK_HIL_roket_modeli(sure_s, birakma_m, port)
% HIL kosumunda kullanilan roket modeli — S-TEK

if nargin < 1 || isempty(sure_s),    sure_s    = 8;  end
if nargin < 2 || isempty(birakma_m), birakma_m = 8;  end
if nargin < 3 || isempty(port),      port      = 'COM8'; end

PORT   = port;
BAUD   = 115200;
TS     = 1/50;
DT     = 1e-3;

g      = 9.81;
KUTLE  = 33.8;

MOMENT_KOL_M = 0.30;
ATALET_KGM2  = 1.995;

ITKI_MAX_N   = 1090;  VANA_TAM_DEG = 90;
K_SERVO_LULE = 0.333;
FW_US_PER_DEG   = 11.11;
PITCH_MERKEZ = (1444.0 - 500)/FW_US_PER_DEG;
ROLL_MERKEZ  = (1500.0 - 500)/FW_US_PER_DEG;
ADIM_PER_DEG = 800/360;

fprintf('=== S-TEK HIL (MATLAB plant) ===\n');
fprintf('  port %s @ %d, alisveris %g Hz, fizik %g ms\n', PORT, BAUD, 1/TS, DT*1e3);
fprintf('  birakma %g m, kutle %.1f kg, sure %g s\n\n', birakma_m, KUTLE, sure_s);

try
    sp = serialport(PORT, BAUD);
catch ME
    fprintf('\n  Mevcut portlar: %s\n', strjoin(cellstr(serialportlist), ', '));
    error(['%s acilamadi: %s\n' ...
           'Dogru portu ucuncu argumanla ver:  STEK_HIL_roket_modeli(30, 2, ''COMx'')'], PORT, ME.message);
end
sp.Timeout = 0.5;
temizle = onCleanup(@() clear('sp'));

fprintf('  kart boot ediliyor (port acilinca resetleniyor)...\n');
pause(2.0);
flush(sp);
fprintf('  hazir, kosu basliyor\n\n');

z = birakma_m;  vz = 0;
th = 0;  dth = 0;
ph = 0;  dph = 0;

N = floor(sure_s/TS);
tv = zeros(N,1);  D = zeros(N,9);  Kk = zeros(N,10);

sonKomut = zeros(1,5);
sonVana = 0; sonSP = 0; sonSY = 0; sonIst = 0; sonDur = 0; sonSeq = 0;
tampon = uint8([]);

kayip = 0;  t = 0;  temasV = [];  temasT = [];
sayac = tic;

for k = 1:N
    q   = euler2quat(ph, th, 0);
    seq = mod(floor(t/0.01), 256);
    write(sp, paketKur(seq, t, q, [dph dth 0], z, vz, KUTLE), 'uint8');

    [vals, tampon, bulundu] = cevapOku(sp, tampon, TS);
    if bulundu
        sonIst  = vals(1);
        sonVana = vals(2)/ADIM_PER_DEG;
        sonSP   = vals(3) - PITCH_MERKEZ;
        sonSY   = vals(4) - ROLL_MERKEZ;
        sonDur  = vals(5);
        sonSeq  = vals(6);
    else
        kayip = kayip + 1;
    end

    itki   = sonVana * (ITKI_MAX_N/VANA_TAM_DEG);
    luleTh = sonSP * K_SERVO_LULE * pi/180;
    lulePh = sonSY * K_SERVO_LULE * pi/180;

    for i = 1:round(TS/DT)
        Fax = itki*cos(luleTh)*cos(lulePh);
        az  = Fax*cos(th)*cos(ph)/KUTLE - g;
        ath = -itki*sin(luleTh)*MOMENT_KOL_M/ATALET_KGM2;
        aph = -itki*sin(lulePh)*MOMENT_KOL_M/ATALET_KGM2;

        vz = vz + az*DT;   z  = z  + vz*DT;
        dth = dth + ath*DT;  th = th + dth*DT;
        dph = dph + aph*DT;  ph = ph + dph*DT;

        if z <= 0
            z = 0;
            if isempty(temasV), temasV = vz; temasT = t + i*DT; end
            vz = 0; dth = 0; dph = 0;
        end
    end

    tv(k)   = t;
    D(k,:)  = [z vz q(1) q(2) q(3) q(4) dph dth 0];
    Kk(k,:) = [itki luleTh sonSY*K_SERVO_LULE sonVana sonSP sonSY sonIst sonDur double(bulundu) sonSeq];

    t = t + TS;

    while toc(sayac) < k*TS
    end

    if ~isempty(temasV) && t > temasT + 1.0
        tv = tv(1:k); D = D(1:k,:); Kk = Kk(1:k,:);
        fprintf('  temas oldu, 1 s sonra durduruldu\n');
        break;
    end
end

gercekSure = toc(sayac);
clear sp

hil_durum.time = tv;   hil_durum.signals.values = D;
hil_komut.time = tv;   hil_komut.signals.values = Kk;
assignin('base','hil_durum',hil_durum);
assignin('base','hil_komut',hil_komut);

n = numel(tv);
fprintf('\n=== KOSU BITTI ===\n');
fprintf('  sim suresi        : %.2f s   (gercek %.2f s, oran %.2fx)\n', tv(end)+TS, gercekSure, (tv(end)+TS)/gercekSure);
fprintf('  alisveris         : %d adet\n', n);
fprintf('  kayip paket       : %d  (%% %.1f)\n', kayip, 100*kayip/n);
fprintf('  en buyuk itki     : %.1f N  (vana %.2f deg)\n', max(Kk(:,1)), max(Kk(:,4)));
if ~isempty(temasV)
    fprintf('  TEMAS             : t=%.2f s, hiz %.3f m/s  %s\n', temasT, temasV, ...
            karar(abs(temasV) < 0.20));
else
    fprintf('  temas olmadi      : son irtifa %.2f m, hiz %.2f m/s\n', z, vz);
end
fprintf('  en buyuk yatis    : %.2f deg\n', max(abs([D(:,8); D(:,7)]))*180/pi);
fprintf('\n  >> STEK_HIL_grafik_uret   ile grafikler ve CSV uretilir\n\n');
end

function s = karar(ok)
    if ok, s = '<-- HEDEF TUTTU'; else, s = '<-- hedef 0.20 m/s'; end
end

function pkt = paketKur(seq, t, q, w, z, vz, mass)
    govde = typecast(single([t q(1) q(2) q(3) q(4) w(1) w(2) w(3) z vz mass]), 'uint8');
    c = uint8(seq);
    for i = 1:numel(govde)
        c = bitxor(c, govde(i));
    end
    pkt = [uint8(170) uint8(85) uint8(seq) govde c];
end

function [vals, tampon, bulundu] = cevapOku(sp, tampon, TS)
    vals = zeros(1,6);  bulundu = false;
    tBas = tic;
    while toc(tBas) < TS*0.9
        na = sp.NumBytesAvailable;
        if na > 0
            tampon = [tampon; uint8(read(sp, na, 'uint8'))'];
        end
        i = 1;  sonBas = 0;
        while i <= numel(tampon)-23
            if tampon(i) == 170 && tampon(i+1) == 85
                c = uint8(0);
                for kk = 2:22
                    c = bitxor(c, tampon(i+kk));
                end
                if c == tampon(i+23)
                    sonBas = i;
                    i = i + 24;
                    continue;
                end
            end
            i = i + 1;
        end
        if sonBas > 0
            v = double(typecast(tampon(sonBas+3:sonBas+22), 'single'));
            vals = [v(:).' double(tampon(sonBas+2))];
            tampon = tampon(sonBas+24:end);
            bulundu = true;
            return;
        end
        if numel(tampon) > 240
            tampon = tampon(end-119:end);
        end
    end
end

function q = euler2quat(roll, pitch, yaw)
    cr = cos(roll/2);  sr = sin(roll/2);
    cp = cos(pitch/2); sp = sin(pitch/2);
    cy = cos(yaw/2);   sy = sin(yaw/2);
    q = [cr*cp*cy + sr*sp*sy, ...
         sr*cp*cy - cr*sp*sy, ...
         cr*sp*cy + sr*cp*sy, ...
         cr*cp*sy - sr*sp*cy];
end
