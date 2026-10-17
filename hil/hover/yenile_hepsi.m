%% yenile_hepsi — iki gorevin grafiklerini kayitli CSV'lerden yeniden uretir
%  Kart takili olmasi gerekmez. matlab -batch ile de calisir.

kok = fileparts(mfilename('fullpath'));
cd(kok);

fprintf('\n########## DIKEY INIS ##########\n');
hil_yukle(fullfile('AHR_DIKEY_INIS','S-TEK-HIL-ayrinti-no-001.csv'));
hil_ciz(1);

fprintf('\n########## HOVER ##########\n');
hil_yukle(fullfile('AHR_HOVER','S-TEK-HIL-ayrinti-no-002.csv'));
hil_ciz(2);

fprintf('\n########## BITTI ##########\n');
