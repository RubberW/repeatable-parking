param([string]$Sketch = 'DigitalTwinParking', [string]$ProcessingRoot = 'E:\Processing')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$javaBin = Join-Path $ProcessingRoot 'app\resources\jdk\bin'
$modeRoot = Join-Path $ProcessingRoot 'app\resources\modes\java\mode'
$coreJar = Join-Path $ProcessingRoot 'app\resources\core\library\core-4.5.7.jar'
$netJar = Join-Path $ProcessingRoot 'app\resources\modes\java\libraries\net\library\net.jar'
$sketchRoot = if ($Sketch -eq 'DigitalTwinBaseline') { Join-Path $projectRoot 'baseline\DigitalTwinBaseline' } else { Join-Path $projectRoot $Sketch }
$buildRoot = Join-Path $projectRoot ('build\' + $Sketch)
New-Item -ItemType Directory -Force -Path $buildRoot, (Join-Path $projectRoot 'build\tools'), (Join-Path $projectRoot 'tmp') | Out-Null
$env:TEMP = Join-Path $projectRoot 'tmp'
$env:TMP = $env:TEMP
& (Join-Path $javaBin 'javac.exe') -cp "$modeRoot\*" -d (Join-Path $projectRoot 'build\tools') (Join-Path $PSScriptRoot 'Preprocess.java')
if ($LASTEXITCODE -ne 0) { throw 'Processing preprocessor helper compilation failed.' }
& (Join-Path $javaBin 'java.exe') -cp "$(Join-Path $projectRoot 'build\tools');$modeRoot\*" Preprocess $sketchRoot $buildRoot
if ($LASTEXITCODE -ne 0) { throw 'Processing preprocessing failed.' }
[string[]]$sources = Get-ChildItem -LiteralPath $buildRoot -Filter '*.java' | ForEach-Object FullName
& (Join-Path $javaBin 'javac.exe') -encoding UTF-8 -cp "$coreJar;$netJar" -d $buildRoot @sources
if ($LASTEXITCODE -ne 0) { throw 'Sketch compilation failed.' }
Write-Output "Built $Sketch using Processing 4.5.7: $buildRoot"
