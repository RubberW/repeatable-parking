# 文件职责：编译唯一的 Processing 草图与共用控制器；生成文件全部放在 .cache。
# 输入：simulation/ParkingSimulation；输出：.cache/classes。由启动器和校验调用。
param([string]$ProcessingRoot='E:\Processing',[switch]$WithChecks)
$ErrorActionPreference='Stop'
$parkingRoot=Split-Path $PSScriptRoot -Parent
$parkingSketch=Join-Path $parkingRoot 'simulation\ParkingSimulation'
$parkingCache=Join-Path $parkingRoot '.cache'
$parkingGenerated=Join-Path $parkingCache 'generated'
$parkingClasses=Join-Path $parkingCache 'classes'
$parkingJava=Join-Path $ProcessingRoot 'app\resources\jdk\bin'
$parkingMode=Join-Path $ProcessingRoot 'app\resources\modes\java\mode'
$parkingCore=Join-Path $ProcessingRoot 'app\resources\core\library\core-4.5.7.jar'
# 每次只编译当前源码，避免旧类或旧生成源码混入。
foreach($parkingFolder in @($parkingGenerated,$parkingClasses)){
 if(Test-Path -LiteralPath $parkingFolder){
  $parkingResolved=(Resolve-Path -LiteralPath $parkingFolder).Path
  if(-not $parkingResolved.StartsWith([IO.Path]::GetFullPath($parkingCache)+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Unexpected build cache path'}
  Remove-Item -LiteralPath $parkingResolved -Recurse -Force
 }
}
New-Item -ItemType Directory -Force $parkingGenerated,$parkingClasses,(Join-Path $parkingCache 'tools'),(Join-Path $parkingCache 'temp') | Out-Null
$env:TEMP=Join-Path $parkingCache 'temp';$env:TMP=$env:TEMP
& (Join-Path $parkingJava 'javac.exe') -encoding UTF-8 -cp "$parkingMode\*" -d (Join-Path $parkingCache 'tools') (Join-Path $PSScriptRoot 'Preprocess.java')
if($LASTEXITCODE -ne 0){throw 'Processing preprocessor compilation failed'}
& (Join-Path $parkingJava 'java.exe') -cp "$(Join-Path $parkingCache 'tools');$parkingMode\*" Preprocess $parkingSketch $parkingGenerated
if($LASTEXITCODE -ne 0){throw 'Processing preprocessing failed'}
$parkingSources=Get-ChildItem -LiteralPath $parkingGenerated -Filter '*.java' | ForEach-Object FullName
& (Join-Path $parkingJava 'javac.exe') -encoding UTF-8 -cp $parkingCore -d $parkingClasses @parkingSources
if($LASTEXITCODE -ne 0){throw 'Current simulation compilation failed'}
if($WithChecks){
 $parkingChecks=Get-ChildItem -LiteralPath (Join-Path $parkingRoot 'experiments\java') -Filter '*.java' | ForEach-Object FullName
 & (Join-Path $parkingJava 'javac.exe') -encoding UTF-8 -cp "$parkingClasses;$parkingCore" -d $parkingClasses @parkingChecks
 if($LASTEXITCODE -ne 0){throw 'Evaluation compilation failed'}
}
Write-Output 'Built one shared Processing simulation and controller.'
