@echo off
setlocal

powershell -ExecutionPolicy Bypass -File "%~dp0flash-script.ps1" %*
set EXIT_CODE=%ERRORLEVEL%

if not "%EXIT_CODE%"=="0" (
  echo.
  echo Flash failed with exit code %EXIT_CODE%.
)

exit /b %EXIT_CODE%
