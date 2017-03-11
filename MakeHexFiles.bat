REM build script v2.0 by Chris Osgood 2013-09-07 http://lunarflow.com/

@echo off
SETLOCAL enabledelayedexpansion

IF "%1"=="" (
   FOR /F "tokens=1,2,*" %%i in (Makefile) DO (
      IF "%%i"=="ALL_TARGETS" (
         FOR %%F IN (%%k) DO (
            call :DoCompile %%~nF
            IF !errorlevel! NEQ 0 goto error
         )
         goto exit
      )
   )
) else (
   FOR %%F IN (%*) DO (
      call :DoCompile %%~nF
      IF !errorlevel! NEQ 0 goto error
   )
)

:exit
pause
goto :eof

:error
echo ********** ERROR **********
pause
goto :eof

:DoCompile
echo.
echo ========== BUILD "%1" ==========
echo.
IF NOT EXIST %1.asm (
   COPY tgy.asm %1.asm
   avra -fI -o %1.inc.hex -D %1_esc -e %1.eeprom -d %1.obj %1.asm
   SET err=!errorlevel!
   DEL %1.asm
) else (
   avra -fI -o %1.inc.hex -D %1_esc -e %1.eeprom -d %1.obj %1.asm
   SET err=!errorlevel!
)
exit /B !err!
