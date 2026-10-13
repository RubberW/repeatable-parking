$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
& (Join-Path $PSScriptRoot 'build.ps1') -Sketch DigitalTwinParkingPrecision
if($LASTEXITCODE -ne 0){throw 'Precision plant build failed'}
$out=Join-Path $root 'build\boundary_heading_cert'
New-Item -ItemType Directory -Force $out | Out-Null
$cp="$root\build\DigitalTwinParkingPrecision;E:\Processing\app\resources\core\library\core-4.5.7.jar;E:\Processing\app\resources\modes\java\libraries\net\library\net.jar"
$sources=Get-ChildItem -LiteralPath (Join-Path $root 'training\boundary_heading_cert\java') -Filter '*.java' | ForEach-Object FullName
& E:\Processing\app\resources\jdk\bin\javac.exe -encoding UTF-8 -cp $cp -d $out @sources
if($LASTEXITCODE -ne 0){throw 'Boundary direction experiment compilation failed'}
Write-Output 'Built boundary direction experiment.'
