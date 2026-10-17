function hil_ciz(deneme_no)
%% hil_ciz — HIL kosusundan AHR grafiklerini ve sartname CSV'sini uretir
%
%  Kullanim:
%     >> hil_ciz          % deneme no 1
%     >> hil_ciz(3)       % deneme no 3
%
%  Girdi  : calisma alanindaki  hil_durum  ve  hil_komut
%           (hil_kur'un ekledigi To Workspace bloklari uretiyor)
%  Cikti  : grafikler/  klasorune 8 PNG + bir CSV
%
%  Sutun duzeni hil_kur.m ile birebir eslesir:
%     hil_durum: 1=z  2=vz  3..6=q(w,x,y,z)  7..9=w(p,q,r)
%     hil_komut: 1=itki_N  2=lule_pitch_rad  3=lule_yaw_deg  4=vana_deg
%                5=pitch_sapma_deg  6=yaw_sapma_deg  7=istenen_N
%                8=durum  9=gecerli

if nargin < 1, deneme_no = 1; end

%% 1) Veriyi al
D = veriAl('hil_durum');
K = veriAl('hil_komut');
if isempty(D) || isempty(K)
    fprintf('\n  Taban calisma alanindaki degiskenler:\n');
    evalin('base','who');
    error(['hil_durum / hil_komut bulunamadi.\n' ...
           'Model kostu mu? Kostuysa yukaridaki listede bir SimulationOutput\n' ...
           'nesnesi (out gibi) olmali. Yoksa To Workspace bloklari veri yazmamis.']);
end

t  = D.time(:);
Dv = degerler(D);
Kv = degerler(K);

if size(Dv,2) < 9 || size(Kv,2) < 9
    fprintf('\n  hil_durum : %d ornek x %d sutun  (9 bekleniyor)\n', size(Dv,1), size(Dv,2));
    fprintf('  hil_komut : %d ornek x %d sutun  (9 bekleniyor)\n', size(Kv,1), size(Kv,2));
    error(['Sutun sayisi tutmuyor. Mux girisleri eksik baglanmis olabilir —\n' ...
           'hil_kur''u tekrar calistirip Ctrl+D yap.']);
end

n = min(numel(t), min(size(Dv,1), size(Kv,1)));
t = t(1:n);  Dv = Dv(1:n,:);  Kv = Kv(1:n,:);

z   = Dv(:,1);      vz  = Dv(:,2);
q   = Dv(:,3:6);    pqr = Dv(:,7:9);

itki_N     = Kv(:,1);   lule_pitch = Kv(:,2)*180/pi;   lule_yaw  = Kv(:,3);
vana_deg   = Kv(:,4);   servo_pitch= Kv(:,5);          servo_yaw = Kv(:,6);
istenen_N  = Kv(:,7);   durum      = Kv(:,8);          gecerli   = Kv(:,9);
if size(Kv,2) >= 10, seq = Kv(:,10); else, seq = nan(size(t)); end

%% 2) Turetilen buyuklukler
ivme = gradient(vz, t);

[roll_d, pitch_d, yaw_d] = quat2euler_zyx(q);

hover_mu = median(durum) >= 1.5;
if hover_mu
    v_ref = hover_referansi(z);
    gorevAdi = 'yerden kalkis (hover)';
else
    v_ref = inis_referansi(z, vz);
    gorevAdi = 'yumusak inis';
end

%% 3) Klasor
kok = fileparts(mfilename('fullpath'));
dz  = fullfile(kok, 'grafikler');
if ~exist(dz,'dir'), mkdir(dz); end

etiket = sprintf('S-TEK HIL — Deneme %03d', deneme_no);

%% 3c) Onemli olay anlari + eksen siniri
olay_t = {}; olay_ad = {};
if hover_mu
    i1 = find(abs(z - 5.0) < 0.10 & abs(vz) < 0.15, 1, 'first');
    if ~isempty(i1), olay_t{end+1} = t(i1); olay_ad{end+1} = 'toleransa giris'; end
    i2 = find(itki_N > 1, 1, 'last');
    if ~isempty(i2), olay_t{end+1} = t(i2);  olay_ad{end+1} = 'gorev sonu'; end
    bitis = i2;  pay = 0.30;  yBitis = i2;
else
    i1 = find(itki_N > 1, 1, 'first');
    if ~isempty(i1), olay_t{end+1} = t(i1); olay_ad{end+1} = 'motor devrede'; end
    i2 = find(z <= 0.005, 1, 'first');
    if ~isempty(i2), olay_t{end+1} = t(i2);  olay_ad{end+1} = 'temas'; end
    bitis = i2;  pay = 0.30;  yBitis = [];
end
tSon = t(end);
if ~isempty(bitis), tSon = min(tSon, t(bitis) + pay); end
xs = [t(1) tSon];

if isempty(yBitis), tPen = tSon; else, tPen = t(yBitis); end
pen = t <= tPen;

%% 4) Grafikler
ci = 0;

ci=ci+1; f=cizim(ci,etiket,'irtifa');
plot(t, z, 'LineWidth',1.4); hold on
if hover_mu
    yline(5.0,'--','hedef irtifa (5 m)','LabelHorizontalAlignment','left','HandleVisibility','off');
else
    yline(5.0,'--','motor devreye (5 m)','LabelHorizontalAlignment','left','HandleVisibility','off');
    yline(0.8,':','kesme (0,8 m)','LabelHorizontalAlignment','left','HandleVisibility','off');
end
xlabel('zaman [s]'); ylabel('irtifa [m]'); title([etiket ' · irtifa–zaman']);
eksen(xs, pen, olay_t, olay_ad, {z});
grid on; kaydet(f, dz, deneme_no, '1_irtifa');

ci=ci+1; f=cizim(ci,etiket,'hiz');
plot(t, vz, 'LineWidth',1.4); hold on
plot(t, v_ref, '--', 'LineWidth',1.2);
legend('gerceklesen v_z','referans v_{ref}','Location','best');
xlabel('zaman [s]'); ylabel('dusey hiz [m/s]');
title([etiket ' · referans–gerceklesen hiz  (' gorevAdi ')']);
eksen(xs, pen, olay_t, olay_ad, {vz, v_ref});
grid on; kaydet(f, dz, deneme_no, '2_hiz_referans');

ci=ci+1; f=cizim(ci,etiket,'ivme');
plot(t, ivme, 'LineWidth',1.2); hold on
yline(-9.81,'--','serbest dusus','HandleVisibility','off');
xlabel('zaman [s]'); ylabel('dusey ivme [m/s^2]');
title([etiket ' · ivme–zaman']); grid on;
eksen(xs, pen, olay_t, olay_ad, {ivme});
kaydet(f, dz, deneme_no, '3_ivme');

ci=ci+1; f=cizim(ci,etiket,'euler');
plot(t, roll_d, t, pitch_d, t, yaw_d, 'LineWidth',1.2);
legend('roll \phi','pitch \theta','yaw \psi','Location','best');
xlabel('zaman [s]'); ylabel('aci [deg]');
title([etiket ' · Euler acilari   (en buyuk sapma ' ...
       vir(max(abs([roll_d(pen); pitch_d(pen); yaw_d(pen)]))) '°)']); grid on;
eksen(xs, pen, olay_t, olay_ad, {roll_d, pitch_d, yaw_d}, [-0.5 0.5]);
kaydet(f, dz, deneme_no, '4_euler');

ci=ci+1; f=cizim(ci,etiket,'pqr');
plot(t, pqr*180/pi, 'LineWidth',1.2);
legend('p','q','r','Location','best');
xlabel('zaman [s]'); ylabel('acisal hiz [deg/s]');
pqr_d = pqr*180/pi;
title([etiket ' · govde acisal hizlari   (en buyuk ' ...
       vir(max(abs(pqr_d(pen,:)),[],'all')) '°/s)']); grid on;
eksen(xs, pen, olay_t, olay_ad, {pqr_d(:,1), pqr_d(:,2), pqr_d(:,3)}, [-0.5 0.5]);
kaydet(f, dz, deneme_no, '5_pqr');

ci=ci+1; f=cizim(ci,etiket,'itki');
plot(t, itki_N, 'LineWidth',1.4);
xlabel('zaman [s]'); ylabel('itki [N]');
title([etiket ' · kontrol girisi: itki']); grid on;
eksen(xs, pen, olay_t, olay_ad, {itki_N});  ylim([-20 640]);
kaydet(f, dz, deneme_no, '6_itki');

ci=ci+1; f=cizim(ci,etiket,'lule');
plot(t, lule_pitch, t, lule_yaw, 'LineWidth',1.2); hold on
yline( 5,'--','doygunluk +5°','LabelHorizontalAlignment','left','HandleVisibility','off');
yline(-5,'--','doygunluk -5°','LabelHorizontalAlignment','left','HandleVisibility','off');
legend('lule pitch','lule yaw','Location','best');
xlabel('zaman [s]'); ylabel('lule acisi [deg]');
title([etiket ' · kontrol girisi: TVC   (en buyuk ' ...
       vir(max(abs([lule_pitch(pen); lule_yaw(pen)]))) '°)']); grid on;
eksen(xs, pen, olay_t, olay_ad, {lule_pitch, lule_yaw}, [-6 6]);
kaydet(f, dz, deneme_no, '7_lule');

ci=ci+1; f=cizim(ci,etiket,'vana');
subplot(2,1,1);
plot(t, istenen_N, '--', t, itki_N, 'LineWidth',1.2); hold on
yline(600,':','600 N doygunluk','HandleVisibility','off');
legend('istenen itki','gerceklesen itki','Location','best');
ylabel('itki [N]'); title([etiket ' · vana ve paket sagligi']); grid on;
xlim(xs); ylim([-20 640]);
subplot(2,1,2);
plot(t, gecerli, 'LineWidth',1.2); hold on
plot(t, durum, 'LineWidth',1.0);
legend('paket gecerli','gorev modu','Location','best');
xlabel('zaman [s]'); xlim(xs); ylim([-0.2 2.2]); grid on;
kaydet(f, dz, deneme_no, '8_vana_paket');

%% 5) Sartname CSV'si
csvad = fullfile(dz, sprintf('S-TEK-HIL-ayrinti-no-%03d.csv', deneme_no));
basliklar = {'saniye','irtifa_m','dusey_hiz_m_s','dusey_ivme_m_s2', ...
             'roll_deg','pitch_deg','yaw_deg','p_deg_s','q_deg_s','r_deg_s', ...
             'itki_N','lule_pitch_deg','lule_yaw_deg', ...
             'itki_istenen_N','vana_gerceklesen_deg', ...
             'servo_pitch_deg','servo_yaw_deg','gorev_modu','paket_gecerli','seq'};
M = [t, z, vz, ivme, roll_d, pitch_d, yaw_d, pqr*180/pi, ...
     itki_N, lule_pitch, lule_yaw, istenen_N, vana_deg, ...
     servo_pitch, servo_yaw, durum, gecerli, seq];

% Ayirici noktali virgul, ondalik virgul: Turkce Excel'de cift tiklayinca
% sutunlara ayrilsin. hil_yukle bu bicimi okuyacak sekilde ayarli.
fid = fopen(csvad, 'w', 'n', 'UTF-8');
if fid < 0, error('CSV acilamadi: %s', csvad); end
fwrite(fid, [239 187 191], 'uint8');
fprintf(fid, '%s', basliklar{1});
fprintf(fid, ';%s', basliklar{2:end});
fprintf(fid, '\n');
bicim = [repmat('%.4f;', 1, size(M,2)-1) '%.4f'];
for i = 1:size(M,1)
    fprintf(fid, '%s\n', strrep(sprintf(bicim, M(i,:)), '.', ','));
end
fclose(fid);

%% 5b) SARTNAME BICIMINDE CSV (Tablo 1 — 6 sutun)
% Sartname 3.2.2-u: dosya adi "Takim Adi-Deneme/Yarisma-Atis no:###",
% Tablo 1 sutunlari, ondalik ayraci virgul. Windows ':' kabul etmedigi
% icin tire kullanildi. Ayirici noktali virgul — virgullu ondalikla
% cakismasin ve Turkce Excel'de sutunlara dogru ayrilsin diye.
sartAd = fullfile(dz, sprintf('S-TEK-Deneme-Atış no-%03d.csv', deneme_no));
fid2 = fopen(sartAd, 'w', 'n', 'UTF-8');
if fid2 > 0
    fwrite(fid2, [239 187 191], 'uint8');
    fprintf(fid2, ['Zaman (sn);İrtifa (m);İniş Hızı (m/s);' ...
                   'Yuvarlanma (Derece);Yunuslama (Derece);Yalpalama (Derece)\n']);
    S = [t, z, -vz, roll_d, pitch_d, yaw_d];
    for i = 1:size(S,1)
        satir = sprintf('%.3f;%.2f;%.2f;%.4f;%.4f;%.4f', S(i,:));
        fprintf(fid2, '%s\n', strrep(satir, '.', ','));
    end
    fclose(fid2);
end

%% 6) Ozet
kayip = sum(gecerli < 0.5);
fprintf('\n=== HIL KOSU OZETI — deneme %03d ===\n', deneme_no);
fprintf('  sure                  : %.2f s  (%d ornek, %.0f Hz)\n', ...
        t(end), n, (n-1)/max(t(end)-t(1), eps));
fprintf('  baslangic irtifasi    : %.2f m\n', z(1));

if hover_mu
    iKes = find(itki_N > 1, 1, 'last');
    iBnt = find(abs(z - 5.0) < 0.10 & abs(vz) < 0.15, 1, 'first');
    fprintf('  hedef irtifa          : 5.00 m\n');
    if ~isempty(iBnt)
        fprintf('  toleransa giris       : t=%.2f s  (irtifa %.3f m)\n', t(iBnt), z(iBnt));
    end
    if ~isempty(iKes)
        fprintf('  GOREV SONU (motor kes): t=%.2f s  (irtifa %.3f m)\n', t(iKes), z(iKes));
        if ~isempty(iBnt)
            fprintf('  tutma suresi          : %.2f s   %s\n', t(iKes)-t(iBnt), ...
                    karar(t(iKes)-t(iBnt) >= 5.0, 'hedef 5 s'));
        end
        tut = itki_N(max(1,iBnt):iKes);
        fprintf('  tutma itkisi (ort)    : %.1f N   (teorik %.1f N)\n', mean(tut), 33.8*9.81);
        fprintf('  gorev sonrasi serbest dusus teste dahil degildir\n');
    end
else
    iT = find(z <= 0.005, 1, 'first');
    if isempty(iT)
        temasZ = z(end);  temasV = vz(end);  indi = false;
    else
        j = iT;
        while j > 1 && vz(j) >= 0, j = j - 1; end
        temasZ = z(iT);   temasV = vz(j);  indi = true;
    end
    fprintf('  temas irtifasi        : %.3f m\n', temasZ);
    fprintf('  temas hizi            : %.3f m/s   %s\n', temasV, hedefDurum(indi, temasV));
end
fprintf('  en buyuk dusey hiz    : %.2f m/s\n', min(vz));
fprintf('  en buyuk itki         : %.1f N  (vana %.1f deg)\n', max(itki_N), max(vana_deg));
fprintf('  en buyuk yatis        : %.2f deg\n', max(abs([roll_d; pitch_d])));
fprintf('  lule doygunlugunda    : %% %.1f\n', 100*mean(abs(lule_pitch) > 4.99));
fprintf('  gecersiz paket        : %d / %d  (%% %.2f)\n', kayip, n, 100*kayip/n);
if any(~isnan(seq))
    ok = gecerli > 0.5 & ~isnan(seq);
    if any(ok)
        tseq  = seq(ok) * 0.01;
        tger  = t(ok);
        sarma = round((tger - tseq) / 2.56);
        gec   = tger - (tseq + sarma*2.56);
        fprintf('  ESP gecikmesi         : ort %.3f s, en buyuk %.3f s\n', mean(gec), max(gec));
        if mean(gec) > 0.05
            fprintf('  [UYARI] kart %.0f ms geride — gordugu durum modelin anlik hali DEGIL.\n', 1000*mean(gec));
        end
    end
end
fprintf('\n  grafikler : %s\n', dz);
fprintf('  csv (detay)    : %s\n', csvad);
fprintf('  csv (sartname) : %s\n\n', sartAd);

if 100*kayip/n > 5
    fprintf('  [UYARI] gecersiz paket %%%.1f — seri hat ya da sagolama sorunu.\n', 100*kayip/n);
end
if ~hover_mu && ~indi
    fprintf('  [UYARI] roket yere inmemis (son irtifa %.2f m) — sure yetmemis olabilir.\n', z(end));
end
end

function s = hedefDurum(indi, v)
    if ~indi
        s = '(inmedi)';
    elseif abs(v) < 0.20
        s = '<-- SARTNAME HEDEFI TUTTU (0.20 alti)';
    else
        s = '<-- hedef 0.20 m/s alti';
    end
end

function s = karar(ok, hedef)
    if ok, s = '<-- HEDEF TUTTU'; else, s = ['<-- ' hedef]; end
end

function M = degerler(S)
    if isnumeric(S)
        M = double(S);
        return;
    end
    if isstruct(S) && isfield(S, 'signals')
        sg = S.signals;
        p  = cell(1, numel(sg));
        for i = 1:numel(sg)
            v = double(sg(i).values);
            if ndims(v) == 3
                v = permute(v, [3 1 2]);
                v = reshape(v, size(v,1), []);
            elseif size(v,1) == 1 && size(v,2) > 1 && numel(sg) > 1
                v = v(:);
            end
            p{i} = v;
        end
        M = [p{:}];
        return;
    end
    M = double(S);
end

function S = veriAl(ad)
    S = [];
    try
        if evalin('base', sprintf('exist(''%s'',''var'')', ad)) == 1
            S = evalin('base', ad);
            return;
        end
    catch
    end
    try
        isimler = evalin('base', 'who');
    catch
        return;
    end
    for i = 1:numel(isimler)
        try
            v = evalin('base', isimler{i});
        catch
            continue;
        end
        if isa(v, 'Simulink.SimulationOutput')
            try
                aday = v.get(ad);
                if ~isempty(aday)
                    S = aday;
                    fprintf('  [bilgi] %s, ''%s'' nesnesinin icinde bulundu\n', ad, isimler{i});
                    return;
                end
            catch
            end
        end
    end
end

function f = cizim(i, ~, ~)
    f = figure(100+i);
    set(f,'Visible','off','Color','w','Position',[80 80 900 520]);
    clf(f);
end

function kaydet(f, dz, no, ad)
    dosya = fullfile(dz, sprintf('deneme%03d_%s.png', no, ad));
    if isempty(f) || ~isvalid(f)
        fprintf('  [ATLANDI] figur kapanmis: %s\n', ad);
        return;
    end
    try
        p = get(f, 'Position');
        set(f, 'PaperUnits','inches', ...
               'PaperPosition',[0 0 p(3)/96 p(4)/96], ...
               'PaperSize',    [p(3)/96 p(4)/96]);
        print(f, dosya, '-dpng', '-r200');
    catch ME
        fprintf('  [ATLANDI] %s kaydedilemedi (%s)\n', ad, ME.message);
    end
end

function [roll_d, pitch_d, yaw_d] = quat2euler_zyx(q)
    w = q(:,1); x = q(:,2); y = q(:,3); zz = q(:,4);
    roll_d  = atan2(2*(w.*x + y.*zz), 1 - 2*(x.^2 + y.^2)) * 180/pi;
    sinp    = max(min(2*(w.*y - zz.*x), 1), -1);
    pitch_d = asin(sinp) * 180/pi;
    yaw_d   = atan2(2*(w.*zz + x.*y), 1 - 2*(y.^2 + zz.^2)) * 180/pi;
end

function s = vir(x)
    s = strrep(sprintf('%.3g', x), '.', ',');
end

function eksen(xs, pen, olay_t, olay_ad, veriler, enAz)

    xlim(xs);

    a = [];
    for i = 1:numel(veriler)
        v = veriler{i}(:);
        v = v(pen(:));
        a = [a; v(isfinite(v))];
    end
    if ~isempty(a)
        lo = min(a);  hi = max(a);
        if nargin >= 6 && ~isempty(enAz)
            lo = min(lo, enAz(1));  hi = max(hi, enAz(2));
        end
        if hi - lo < 1e-9, lo = lo - 0.5;  hi = hi + 0.5;  end
        m = 0.10*(hi - lo);
        ylim([lo-m hi+m]);
    end

    bicim = {'--', ':'};
    for i = 1:numel(olay_t)
        xline(olay_t{i}, bicim{min(i,2)}, olay_ad{i}, ...
              'LabelVerticalAlignment','bottom', 'LabelOrientation','horizontal', ...
              'Color',[0.35 0.35 0.35], 'HandleVisibility','off');
    end
end

function vref = hover_referansi(z)
    Z_HEDEF = 5.0;   Z_BAS = 2.0;   V_TIRMAN = 0.50;
    vref = zeros(size(z));
    for i = 1:numel(z)
        if z(i) >= Z_HEDEF
            vref(i) = 0.0;
        elseif z(i) <= Z_BAS
            vref(i) = V_TIRMAN;
        else
            vref(i) = (Z_HEDEF - z(i)) / (Z_HEDEF - Z_BAS) * V_TIRMAN;
        end
    end
end

function vref = inis_referansi(z, vz)
    Z_ACT   = 5.00;   Z_CUT    = 0.80;
    Z_FLARE = 4.90;   Z_TOUCH  = 0.25;
    V_TOUCH = -0.15;
    VZ_MIN  = -0.25;  VZ_MAX   = 0.05;

    n = numel(z);
    vref = zeros(n,1);
    v_entry = -8.86;

    for i = 1:n
        zi = max(z(i), 0);   vi = vz(i);

        if zi > Z_ACT
            v_entry = vi;
            vref(i) = 0;
        elseif zi <= Z_CUT && vi >= VZ_MIN && vi <= VZ_MAX
            vref(i) = 0;
        elseif zi > Z_FLARE
            vref(i) = v_entry;
        elseif zi > Z_TOUCH
            payda = max(Z_FLARE - Z_TOUCH, 1e-4);
            oran  = min(max((zi - Z_TOUCH)/payda, 0), 1);
            vref(i) = V_TOUCH + oran*(v_entry - V_TOUCH);
        else
            vref(i) = V_TOUCH;
        end
    end
end
