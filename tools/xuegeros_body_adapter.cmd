@echo off
setlocal
set XUEGEROS_PREVIEW_ENABLE=1
python "%~dp0xuegeros_body_adapter.py"
exit /b %ERRORLEVEL%
