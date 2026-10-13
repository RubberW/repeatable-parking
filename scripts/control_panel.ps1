$ErrorActionPreference='Stop'
& (Join-Path $PSScriptRoot 'current_demo.ps1')
if($LASTEXITCODE -ne 0){throw 'Retained control panel failed'}
