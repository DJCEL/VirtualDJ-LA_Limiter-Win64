@echo off
rem Run from a "Developer Command Prompt for VS" (cl.exe must be in the PATH).
cd /d "%~dp0"
cl /nologo /std:c++20 /O2 /DNDEBUG /EHsc /I..\src test_limiter2.cpp ..\src\Limiter2.cpp /Fe:test_limiter2.exe || exit /b 1
test_limiter2.exe
exit /b %ERRORLEVEL%
