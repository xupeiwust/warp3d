@echo off
cls
::
::
:: ****************************************************************************
::
::                             Makewarp_jom.bat cmd file
::   
::   Usage:   Makewarp_jom.bat executed in a Command shell
::
::
::   This version requires jom.exe to support concurrent compiles using
::   the standard .nmake file for building WARP3D.
::
::   jom is a replacement for nmake
::
::   The WARP3D src/Makefile.windows.nmake works with both the usual 
::   "nmake" command and the "jom" replacement for nmake.
:: 
::   http://wiki.qt.io/Jom    where to find Jom
::
::   This script builds a 64-bit version of WARP3D.
::   It must be executed from within a command shell that has environment
::   variables initialed to use the 64-bit version of the Intel ifx
::   compiler suite which includes the MKL math library.
::
::   This set up works ** only ** for Intel ifx compiler.
::
:: ****************************************************************************
::
::
::       Location of the "jom" tool must be set here.
::       Number of concurrent compiles to be allowed
::
set jom_exe="c:\Users\rdodds\bin\jom_1_1_2\jom.exe"
set jobs="7"
::
if not exist %jom_exe% (
  echo.
  echo .... Fatal Error: the jom.exe tool must be available
  echo ....              set the shell variable inside this script to
  echo ....              its location.
  echo ....              http://wiki.qt.io/Jom
  goto done
  echo.
 )
set build_mode=64
::
::
  echo.
  echo    ******************************************************
  echo    *                                                    *
  echo    *       Makewarp batch file for Windows              *
  echo    *             (64-bit architectures)                 *
  echo    *                                                    *
  echo    ******************************************************
  echo.
::
::   ==================================================================
::
::	Create directories for object code and executable files as needed
::
  if not exist ..\run_windows (
     md ..\run_windows
     echo -- Making windows executable file directory...
     echo.
     )
  if not exist ..\obj_windows (
     md ..\obj_windows
     echo -- Making windows object file directory...
     echo.
     )
::
::   ==================================================================
::
::	Run the makefile.
::
::  touch main so it always gets compiled. internally gets date/time
::  of this compile and includes in warp3d header block
::
copy /b main_program.f +,,
::
if "%build_mode%" == "64" (
  echo -- Compiling WARP3D for 64-bit execution on
  echo -- Windows with jom utility...
  echo.
  %jom_exe% /f Makefile.windows.nmake ARCH64=64 /J %jobs%
 )
  del /Q /F .\*.pdb .\*.exe
::
  echo.
  echo.
  echo -- WARP3D updated. You may need to increase the
  echo    virtual memory limits on Windows implementations.
  echo    Refer to the README file in the main WARP3D directory...
  echo.
  echo -- All done...
  echo.
:done

