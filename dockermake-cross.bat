@echo off
set OLDCWD=%cd%
cd %~dp0
set ROOT=%cd%
cd %OLDCWD%

set REL=%OLDCWD%

SETLOCAL EnableDelayedExpansion

set MAT=
set UPP=
for /f "tokens=*" %%a in ('echo.%ROOT:\=^&echo.%') do (
    set SUB=!SUB!%%a\
    call set TMP=%%src:!SUB!=%%
    set MAT=!SUB!
)
set REL=%UPP%!REL:%MAT%=!
if "!REL!" EQU "!ROOT!" (set REL=) ELSE (set "REL=!REL:\=/!")
set REL=/!REL!

docker pull caldog20/aarch64cc:latest
docker run --rm --env-file "%ROOT%/env.list" -i -w"/project%REL%" -v "%ROOT%:/project" caldog20/aarch64cc:latest make --makefile=Makefile-CROSS %*
