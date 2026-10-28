; Script do Inno Setup para gerar um instalador Windows (Setup.exe) do
; Inclinometro Avibras Aeroco a partir do executavel ja empacotado pelo
; PyInstaller (windows\build_exe.bat).
;
; Requer o Inno Setup instalado (https://jrsoftware.org/isinfo.php) e que
; "dist\Inclinometro2Eixos\Inclinometro2Eixos.exe" ja exista (rode
; build_exe.bat antes).
; Gerar com: windows\build_installer.bat

; ATENCAO AO AppId: e' ele que o Windows usa para decidir se uma instalacao
; e' um app novo ou um UPGRADE de outro ja instalado. Este GUID e' diferente
; do da versao 1 (so inclinacao), e e' isso que permite ter as duas
; instaladas ao mesmo tempo, cada uma com sua entrada em "Adicionar ou
; remover programas". Nome, pasta e atalhos tambem sao proprios pelo mesmo
; motivo. Nunca reaproveitar o GUID da outra versao.
;
; Manter em sincronia com python-app\app_version.py (mesmos nome e versao).

#define MyAppName "Inclinometro 2 Eixos (Avibras Aeroco)"
#define MyAppVersion "2.2.0"
#define MyAppPublisher "Avibras Aeroco"
#define MyAppExeName "Inclinometro2Eixos.exe"

[Setup]
AppId={{3F7A2C5D-8E14-4D6B-A2F9-5C7B1E0A9D34}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\Inclinometro2Eixos
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputDir=installer_output
OutputBaseFilename=Inclinometro-2Eixos-Setup-{#MyAppVersion}
Compression=lzma
SolidCompression=yes
SetupIconFile=..\assets\logo.ico
UninstallDisplayIcon={app}\{#MyAppExeName}
ArchitecturesInstallIn64BitMode=x64compatible
WizardStyle=modern

[Languages]
Name: "brazilianportuguese"; MessagesFile: "compiler:Languages\BrazilianPortuguese.isl"

[Tasks]
Name: "desktopicon"; Description: "Criar atalho na Area de Trabalho"; GroupDescription: "Atalhos adicionais:"

[Files]
Source: "..\dist\Inclinometro2Eixos\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\Desinstalar {#MyAppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Abrir {#MyAppName} agora"; Flags: nowait postinstall skipifsilent
