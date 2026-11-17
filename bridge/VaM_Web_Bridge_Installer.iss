#define MyAppName "VaM Web Bridge"
#define MyAppVersion "1.0.0"
#define MyAppExeName "VaM_Web_Bridge.exe"

[Setup]
AppId={{AA3A6C21-DC92-45AD-B833-F3F75D70EA1C}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher=VaM Web Community
DefaultDirName={localappdata}\Programs\VaM Web Bridge
DefaultGroupName=VaM Web Bridge
OutputDir=installer_output
OutputBaseFilename=VaM_Web_Bridge_Setup_1.0.0
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=lowest
UninstallDisplayIcon={app}\{#MyAppExeName}
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

[Files]
Source: "dist\VaM_Web_Bridge\*"; DestDir: "{app}"; Excludes: "bridge_config.json"; Flags: recursesubdirs createallsubdirs ignoreversion
Source: "dist\VaM_Web_Bridge\bridge_config.json"; DestDir: "{app}"; Flags: onlyifdoesntexist uninsneveruninstall

[Icons]
Name: "{group}\VaM Web Bridge"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"
Name: "{group}\Uninstall VaM Web Bridge"; Filename: "{uninstallexe}"
Name: "{userdesktop}\VaM Web Bridge"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Additional shortcuts:"

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch VaM Web Bridge"; Flags: postinstall nowait skipifsilent unchecked
