[CmdletBinding()]
param(
    [switch]$BuildFirst,
    [switch]$ForceCloseUV4
)

$ErrorActionPreference = "Stop"

$repoRoot = $PSScriptRoot
$projectDir = Join-Path $repoRoot "project\MDK_V5"
$projectFile = Join-Path $projectDir "AT32F403ACCT7_WorkBench.uvprojx"
$keilExe = "E:\tools\keil5\UV4\UV4.exe"
$objectsDir = Join-Path $projectDir "objects"
$axfFile = Join-Path $objectsDir "AT32F403ACCT7_WorkBench.axf"
$flashLog = Join-Path $objectsDir "AT32F403ACCT7_WorkBench.flash_log.txt"
$buildScript = Join-Path $repoRoot "build-script.ps1"

function Write-Section {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Title
    )

    Write-Host ""
    Write-Host "${Title}:" -ForegroundColor Cyan
}

function Write-TextLog {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }

    $lines = Get-Content -LiteralPath $Path
    if ($lines.Count -eq 0) {
        return
    }

    Write-Section -Title "Flash log"
    $lines | ForEach-Object { Write-Host $_ }
}

function Format-FlashLogText {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    $formatted = $Text -replace '\r?\n', "`n"
    $formatted = $formatted -replace '(?<!\r?\n)(Erase Done\.)', "`n`$1"
    $formatted = $formatted -replace '(?<!\r?\n)(Programming Done\.)', "`n`$1"
    $formatted = $formatted -replace '(?<!\r?\n)(Verify OK\.)', "`n`$1"
    $formatted = $formatted -replace '(?<!\r?\n)(Application running \.\.\.)', "`n`$1"
    $formatted = $formatted -replace '(?<!\r?\n)(Flash Load finished.*)', "`n`$1"
    $formatted = $formatted -replace '^\s+', ''

    return $formatted
}

function Write-FlashProgress {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    $normalized = Format-FlashLogText -Text $Text
    if ([string]::IsNullOrWhiteSpace($normalized)) {
        return
    }

    Write-Section -Title "Flash log"
    $normalized -split "`n" |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_.Length -gt 0 } |
        ForEach-Object { Write-Host $_ }
}

if (-not (Test-Path -LiteralPath $keilExe)) {
    Write-Error "Keil executable not found: $keilExe"
}

if (-not (Test-Path -LiteralPath $projectFile)) {
    Write-Error "Project file not found: $projectFile"
}

if ($BuildFirst) {
    Write-Host "Building before flashing..." -ForegroundColor Cyan
    $buildArgs = @("-ExecutionPolicy", "Bypass", "-File", $buildScript)
    if ($ForceCloseUV4) {
        $buildArgs += "-ForceCloseUV4"
    }

    & powershell.exe @buildArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if (-not (Test-Path -LiteralPath $axfFile)) {
    Write-Error "AXF file not found: $axfFile. Run .\\build first or use -BuildFirst."
}

$uv4Processes = Get-Process UV4 -ErrorAction SilentlyContinue
if ($uv4Processes) {
    if ($ForceCloseUV4) {
        $uv4Processes | Stop-Process -Force
        Start-Sleep -Seconds 1
    }
    else {
        Write-Host "Detected running UV4 process. Flashing may fail if that session still owns the project or debugger." -ForegroundColor Yellow
        Write-Host "Use -ForceCloseUV4 if you want the script to close UV4 before flashing." -ForegroundColor Yellow
    }
}

if (Test-Path -LiteralPath $flashLog) {
    Remove-Item -LiteralPath $flashLog -Force
}

Write-Host "Flashing with Keil..." -ForegroundColor Cyan
Write-Host "Project: $projectFile"
Write-Host "AXF: $axfFile"

Push-Location $projectDir
try {
    & $keilExe -f $projectFile -o $flashLog
}
finally {
    Pop-Location
}

$lastPrintedOutput = ""

for ($attempt = 0; $attempt -lt 20; $attempt++) {
    if (Test-Path -LiteralPath $flashLog) {
        $logItem = Get-Item -LiteralPath $flashLog
        if ($logItem.Length -gt 0) {
            $lastPrintedOutput = Get-Content -LiteralPath $flashLog -Raw
            break
        }
    }

    Start-Sleep -Milliseconds 500
}

if (-not (Test-Path -LiteralPath $flashLog)) {
    Write-Host "Flash log was not generated: $flashLog" -ForegroundColor Red
    exit 5
}

$flashOutput = ""
$stableCount = 0
$previousLength = -1

for ($attempt = 0; $attempt -lt 30; $attempt++) {
    $flashOutput = Get-Content -LiteralPath $flashLog -Raw
    $currentLength = $flashOutput.Length

    if ($flashOutput -ne $lastPrintedOutput) {
        $lastPrintedOutput = $flashOutput
    }

    if ($flashOutput -match 'Verify OK\.' -or
        $flashOutput -match 'Flash Load finished' -or
        $flashOutput -match 'Application running \.\.\.' -or
        $flashOutput -match 'No Algorithm found' -or
        $flashOutput -match 'Error:' -or
        $flashOutput -match 'Cannot Load Flash Device Description') {
        break
    }

    if ($currentLength -eq $previousLength) {
        $stableCount++
    }
    else {
        $stableCount = 0
        $previousLength = $currentLength
    }

    if ($stableCount -ge 3) {
        break
    }

    Start-Sleep -Milliseconds 500
}

Write-FlashProgress -Text $flashOutput

if ($flashOutput -match 'Flash Load finished' -or
    ($flashOutput -match 'Programming Done\.' -and $flashOutput -match 'Verify OK\.')) {
    Write-Host "Flash succeeded." -ForegroundColor Green
    Write-Host "Log: $flashLog"
    exit 0
}

Write-Host "Flash may have failed or did not complete successfully." -ForegroundColor Red
Write-Host "Log: $flashLog"
exit 1
