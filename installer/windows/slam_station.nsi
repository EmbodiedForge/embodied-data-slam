; SLAM Station Windows x64 Installer (NSIS 3, zlib/libpng license - commercial friendly)
; Bundles SLAM Station (Dear ImGui), umi_slam_pipeline, ORB-SLAM3 native backend,
; device/calibration profiles, and Microsoft Visual C++ Redistributable (vc_redist.x64.exe).

Unicode true

!include "MUI2.nsh"
!include "x64.nsh"

!ifndef APP_VERSION
  !define APP_VERSION "2.0.0"
!endif
!ifndef DIST_DIR
  !define DIST_DIR "..\..\dist\SLAMStation"
!endif
!ifndef OUT_FILE
  !define OUT_FILE "..\..\dist\SLAMStation-Setup-${APP_VERSION}-x64.exe"
!endif

!define APP_NAME "SLAM Station"
!define APP_PUBLISHER "Embodied Data SLAM"
!define APP_UNINST_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\SLAMStation"

Name "${APP_NAME} ${APP_VERSION}"
OutFile "${OUT_FILE}"
InstallDir "$PROGRAMFILES64\SLAMStation"
InstallDirRegKey HKLM "${APP_UNINST_KEY}" "InstallLocation"
RequestExecutionLevel admin
SetCompressor /SOLID lzma

!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN "$INSTDIR\slam_station.exe"
!define MUI_FINISHPAGE_RUN_TEXT "Launch SLAM Station"
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

!insertmacro MUI_LANGUAGE "SimpChinese"
!insertmacro MUI_LANGUAGE "English"

Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_OK|MB_ICONSTOP "SLAM Station requires 64-bit Windows (x64)."
    Abort
  ${EndIf}
  SetRegView 64
FunctionEnd

Section "SLAM Station (Core + Native Backend)" SecCore
  SectionIn RO
  SetOutPath "$INSTDIR"

  ; Install all staged application, pipeline, backend, devices, and profiles
  File /r /x "redist" "${DIST_DIR}\*.*"

  ; Install Microsoft Visual C++ x64 Redistributable silently
  DetailPrint "Installing Microsoft Visual C++ x64 Redistributable..."
  SetOutPath "$TEMP"
  File "${DIST_DIR}\redist\vc_redist.x64.exe"
  ExecWait '"$TEMP\vc_redist.x64.exe" /install /quiet /norestart' $0
  Delete "$TEMP\vc_redist.x64.exe"
  DetailPrint "vc_redist.x64.exe finished with exit code $0"

  ; Start Menu shortcuts
  SetOutPath "$INSTDIR"
  CreateDirectory "$SMPROGRAMS\${APP_NAME}"
  CreateShortCut "$SMPROGRAMS\${APP_NAME}\SLAM Station.lnk" "$INSTDIR\slam_station.exe" "" "$INSTDIR\slam_station.exe" 0
  CreateShortCut "$SMPROGRAMS\${APP_NAME}\SLAM Calibrator.lnk" "$INSTDIR\slam_station.exe" "--calibrator" "$INSTDIR\slam_station.exe" 0
  CreateShortCut "$SMPROGRAMS\${APP_NAME}\Uninstall SLAM Station.lnk" "$INSTDIR\Uninstall.exe"

  ; Uninstaller & Add/Remove Programs metadata
  WriteUninstaller "$INSTDIR\Uninstall.exe"
  SetRegView 64
  WriteRegStr HKLM "${APP_UNINST_KEY}" "DisplayName" "${APP_NAME} ${APP_VERSION}"
  WriteRegStr HKLM "${APP_UNINST_KEY}" "DisplayVersion" "${APP_VERSION}"
  WriteRegStr HKLM "${APP_UNINST_KEY}" "Publisher" "${APP_PUBLISHER}"
  WriteRegStr HKLM "${APP_UNINST_KEY}" "InstallLocation" "$INSTDIR"
  WriteRegStr HKLM "${APP_UNINST_KEY}" "UninstallString" '"$INSTDIR\Uninstall.exe"'
  WriteRegStr HKLM "${APP_UNINST_KEY}" "DisplayIcon" "$INSTDIR\slam_station.exe"
  WriteRegDWORD HKLM "${APP_UNINST_KEY}" "NoModify" 1
  WriteRegDWORD HKLM "${APP_UNINST_KEY}" "NoRepair" 1
SectionEnd

Section "Desktop Shortcut (SLAM Station)" SecDesktop
  SetOutPath "$INSTDIR"
  CreateShortCut "$DESKTOP\SLAM Station.lnk" "$INSTDIR\slam_station.exe" "" "$INSTDIR\slam_station.exe" 0
SectionEnd

Section /o "Desktop Shortcut (SLAM Calibrator)" SecCalibratorDesktop
  SetOutPath "$INSTDIR"
  CreateShortCut "$DESKTOP\SLAM Calibrator.lnk" "$INSTDIR\slam_station.exe" "--calibrator" "$INSTDIR\slam_station.exe" 0
SectionEnd

Section "Uninstall"
  SetRegView 64
  Delete "$DESKTOP\SLAM Station.lnk"
  Delete "$DESKTOP\SLAM Calibrator.lnk"
  RMDir /r "$SMPROGRAMS\${APP_NAME}"
  RMDir /r "$INSTDIR\backend"
  RMDir /r "$INSTDIR\devices"
  RMDir /r "$INSTDIR\profiles"
  Delete "$INSTDIR\slam_station.exe"
  Delete "$INSTDIR\umi_slam_pipeline.exe"
  Delete "$INSTDIR\DEVICE_API.md"
  Delete "$INSTDIR\*.dll"
  Delete "$INSTDIR\Uninstall.exe"
  RMDir "$INSTDIR"
  DeleteRegKey HKLM "${APP_UNINST_KEY}"
SectionEnd
