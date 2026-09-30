<#
.SYNOPSIS
Copies the spatial and room vocabularies beside room models that import them.

.DESCRIPTION
HyMeKo imports resolve relative to the importing file, so a room model kept in another
repository carries copies of meta_spatial.hymeko and meta_room.hymeko. This directory's
data/webxr files remain the originals; consumers compare their copies with
hymeko_webxr::{META_SPATIAL, META_ROOM} in a test. Run this after changing either vocabulary.

.EXAMPLE
./sync-vocabulary.ps1 -Destination ../../../../xrobotassist_workspace/middleware_hymeko/examples
#>
param([Parameter(Mandatory)][string[]]$Destination)
$ErrorActionPreference = 'Stop'
$source = Join-Path $PSScriptRoot '..\data\webxr'
foreach ($target in $Destination) {
    if (-not (Test-Path -LiteralPath $target -PathType Container)) { throw "Missing destination directory: $target" }
    foreach ($name in 'meta_spatial.hymeko', 'meta_room.hymeko') {
        Copy-Item -LiteralPath (Join-Path $source $name) -Destination (Join-Path $target $name) -Force
        Write-Host "$name -> $((Resolve-Path -LiteralPath $target).Path)"
    }
}
