@echo off
setlocal

REM === ABSOLUUTNE BAASTEE (VAJADUSEL MUUDA AINULT SEE RIDA) ===
set "BASE=C:\Users\Kasutaja\Desktop\Pico_2026"

REM === ÄRA MUUDA ALLPOOL OLEVAID RIDU, KUI KAUSTAD ON NAGU MEIL ===
set "COOLTERM=%BASE%\CoolTermWin64Bit\CoolTerm.exe"
set "LOGDIR=%BASE%\logs"
set "LOGFILE=%LOGDIR%\coolterm_log.txt"
set "PS1=%BASE%\scripts\PicoMuon-CoolTerm-Watch-5s.ps1"

echo [INFO] BASE     : %BASE%
echo [INFO] CoolTerm : %COOLTERM%
echo [INFO] LogFile  : %LOGFILE%
echo [INFO] Watcher  : %PS1%
echo.

REM Loo logikaust ja -fail, kui puudu
if not exist "%LOGDIR%" mkdir "%LOGDIR%"
if not exist "%LOGFILE%" type nul > "%LOGFILE%"

REM Kontrollid
if not exist "%COOLTERM%" (
  echo [VIGA] Ei leia CoolTermi: %COOLTERM%
  pause & exit /b 1
)
if not exist "%PS1%" (
  echo [VIGA] Ei leia watcherit: %PS1%
  pause & exit /b 1
)

REM 1) Käivita CoolTerm (kasutab sinu "Save As Default" seadeid)
start "" "%COOLTERM%"

REM 2) Oota 5s, et aken avaneks (vajadusel vajuta Connect)
powershell -NoProfile -Command "Start-Sleep -Seconds 5"

REM 3) Käivita watcher: 1s bin; ainult C; 25s aken; piiks + C-rea echo
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%" -LogPath "%LOGFILE%" -BinSeconds 1 -InactivitySeconds 30 -CountMode C -WindowSeconds 25 -BeepOnMuon -EchoMuon -EchoAll

echo.
echo [VALMIS] Väljundid on kaustas: %BASE%
echo   - pico_muon_events.csv
echo   - pico_muon_hist_1s.csv
echo   - pico_muon_hist_1s.png
pause
endlocal