param([long]$Seed=2026101627,[ValidateSet(1,2,3,4)][int]$Size=3,[switch]$SkipBuild)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$chosen=Get-Content -LiteralPath (Join-Path $root 'configs\CURRENT_SEARCH.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$policy=Join-Path $root $chosen.policy
if($chosen.launcher -ne 'heading' -or $chosen.local_method -ne 'early_fit'){throw 'Unexpected retained module configuration'}
if((Get-FileHash -LiteralPath $policy -Algorithm SHA256).Hash.ToLowerInvariant() -ne $chosen.sha256){throw 'Retained policy checksum mismatch'}
& (Join-Path $PSScriptRoot 'boundary_heading_demo.ps1') -Seed $Seed -Size $Size -SkipBuild:$SkipBuild
if($LASTEXITCODE -ne 0){throw 'Current simulation failed'}
