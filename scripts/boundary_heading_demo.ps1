param([long]$Seed=2026101627,[ValidateSet(1,2,3,4)][int]$Size=3,[switch]$SkipBuild)
$ErrorActionPreference='Stop'
if(-not $SkipBuild){
 & (Join-Path $PSScriptRoot 'build_boundary_heading_cert.ps1')
 if($LASTEXITCODE -ne 0){throw 'Retained controller build failed'}
}
$root=Split-Path $PSScriptRoot -Parent
& python (Join-Path $root 'training\boundary_heading_cert\panel_experiment.py') --size $Size --seed $Seed
if($LASTEXITCODE -ne 0){throw 'Retained control panel failed'}
