# 文件职责：唯一启动脚本，直接运行我们自己的 Processing 停车仿真。
# 输入：场地尺寸、随机种子；输出：仿真窗口。由停车控制台.cmd调用。
param([ValidateSet(1,2,3,4)][int]$Size=3,[long]$Seed=20261014,[switch]$SkipBuild,[ValidateRange(0,120)][int]$CheckFrames=0)
$ErrorActionPreference='Stop'
$parkingRoot=Split-Path $PSScriptRoot -Parent
if(-not $SkipBuild){& (Join-Path $PSScriptRoot 'build.ps1');if($LASTEXITCODE -ne 0){throw 'Build failed'}}
$parkingCP="$(Join-Path $parkingRoot '.cache\classes');E:\Processing\app\resources\core\library\core-4.5.7.jar"
# 检查时渲染指定帧数后退出；正常双击为0，窗口持续运行。
& E:\Processing\app\resources\jdk\bin\java.exe '-Xmx512m' "-Dparking.root=$parkingRoot" "-Dparking.arenaMetres=$Size" "-Dparking.demoSeed=$Seed" "-Dparking.smokeFrames=$CheckFrames" -cp $parkingCP ParkingSimulation
if($LASTEXITCODE -ne 0){throw 'Simulation failed'}
