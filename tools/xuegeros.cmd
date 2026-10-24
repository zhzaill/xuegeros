@echo off
setlocal
pwsh -NoProfile -File "%~dp0xuegeros.ps1" %*
exit /b %ERRORLEVEL%
