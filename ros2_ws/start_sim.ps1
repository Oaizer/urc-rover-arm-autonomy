param(
    [switch]$NoBuild,
    [switch]$Headless,
    [switch]$NoAutoDemo,
    [switch]$SoftwareRendering
)
$ErrorActionPreference = 'Stop'
$windowsWorkspace = $PSScriptRoot.Replace('\', '/')
$resolvedPath = & wsl.exe -d Ubuntu-24.04 --exec wslpath -a $windowsWorkspace
if ($LASTEXITCODE -ne 0) { throw 'Cannot access Ubuntu-24.04 in WSL.' }
$linuxWorkspace = $resolvedPath.Trim()
$launchArgs = @()
if ($Headless) { $launchArgs += 'rviz:=false' }
if ($NoAutoDemo) { $launchArgs += 'auto_demo:=false' }
$buildOption = if ($NoBuild) { '1' } else { '0' }
$renderOption = if ($SoftwareRendering) { '1' } else { '0' }
& wsl.exe -d Ubuntu-24.04 --exec env "URC_SKIP_BUILD=$buildOption" "URC_SOFTWARE_RENDERING=$renderOption" bash "$linuxWorkspace/scripts/run_demo.sh" @launchArgs
if ($LASTEXITCODE -notin @(0, 130)) { throw "Linux simulator exited with code $LASTEXITCODE. Check the output above." }
