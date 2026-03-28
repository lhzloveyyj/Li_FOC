[CmdletBinding()]
param(
    [switch]$ForceCloseUV4
)

$ErrorActionPreference = "Stop"

$repoRoot = $PSScriptRoot
$projectDir = Join-Path $repoRoot "project\MDK_V5"
$projectFile = Join-Path $projectDir "AT32F403ACCT7_WorkBench.uvprojx"
$keilExe = "E:\tools\keil5\UV4\UV4.exe"
$objectsDir = Join-Path $projectDir "objects"
$buildLog = Join-Path $objectsDir "AT32F403ACCT7_WorkBench.build_log.htm"
$axfFile = Join-Path $objectsDir "AT32F403ACCT7_WorkBench.axf"
$hexFile = Join-Path $objectsDir "AT32F403ACCT7_WorkBench.hex"

function Get-BuildLogLines {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Content
    )

    $text = $Content -replace '(?is)<script.*?</script>', ''
    $text = $text -replace '(?is)<style.*?</style>', ''
    $text = $text -replace '(?i)<br\s*/?>', "`n"
    $text = $text -replace '(?i)</(pre|h1|h2|p|div|body|html)>', "`n"
    $text = $text -replace '(?is)<[^>]+>', ''
    $text = $text.Replace('&lt;', '<').Replace('&gt;', '>').Replace('&amp;', '&').Replace('&quot;', '"')

    return $text -split "`r?`n" |
        ForEach-Object { $_.TrimEnd() } |
        Where-Object { $_.Trim().Length -gt 0 }
}

function Write-BuildDiagnostics {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Lines,
        [Parameter(Mandatory = $true)]
        [ValidateSet("error", "warning")]
        [string]$Kind
    )

    $matcher = if ($Kind -eq "error") { ':\s*error:' } else { ':\s*warning:' }
    $diagnosticLines = @()

    for ($i = 0; $i -lt $Lines.Count; $i++) {
        if ($Lines[$i] -match $matcher) {
            $diagnosticLines += [string]$Lines[$i]

            if ($i + 1 -lt $Lines.Count) {
                $nextLine = $Lines[$i + 1]
                if ($nextLine -match '^\s' -or $nextLine -match '^[A-Za-z_][A-Za-z0-9_]*$') {
                    $diagnosticLines += ("  $nextLine".TrimEnd())
                }
            }
        }
    }

    if ($diagnosticLines.Count -gt 0) {
        $title = if ($Kind -eq "error") { "Errors" } else { "Warnings" }
        Write-Host ""
        Write-Host "${title}:" -ForegroundColor Yellow
        $diagnosticLines | Select-Object -Unique | ForEach-Object { Write-Host $_ }
    }
}

function Write-FullBuildLog {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Lines
    )

    if ($Lines.Count -eq 0) {
        return
    }

    Write-Host ""
    Write-Host "Build log:" -ForegroundColor Cyan
    $Lines | ForEach-Object { Write-Host $_ }
}

if (-not (Test-Path -LiteralPath $keilExe)) {
    Write-Error "Keil executable not found: $keilExe"
}

if (-not (Test-Path -LiteralPath $projectFile)) {
    Write-Error "Project file not found: $projectFile"
}

$uv4Processes = Get-Process UV4 -ErrorAction SilentlyContinue
if ($uv4Processes) {
    if ($ForceCloseUV4) {
        $uv4Processes | Stop-Process -Force
        Start-Sleep -Seconds 1
    }
    else {
        Write-Host "Detected running UV4 process. Continuing build without closing Keil." -ForegroundColor Yellow
        Write-Host "Use -ForceCloseUV4 only when you explicitly want the script to close all UV4 instances first." -ForegroundColor Yellow
    }
}

Write-Host "Building Keil project..." -ForegroundColor Cyan
Write-Host "Project: $projectFile"

$previousBuildLogTime = $null
if (Test-Path -LiteralPath $buildLog) {
    $previousBuildLogTime = (Get-Item -LiteralPath $buildLog).LastWriteTime
}

Push-Location $projectDir
try {
    & $keilExe -b $projectFile -j0
}
finally {
    Pop-Location
}

if (-not (Test-Path -LiteralPath $buildLog)) {
    Write-Error "Build log was not generated: $buildLog"
}

$buildLogItem = $null
$freshLogDetected = $false

for ($attempt = 0; $attempt -lt 10; $attempt++) {
    $buildLogItem = Get-Item -LiteralPath $buildLog
    if (($null -eq $previousBuildLogTime) -or ($buildLogItem.LastWriteTime -gt $previousBuildLogTime)) {
        $freshLogDetected = $true
        break
    }

    Start-Sleep -Milliseconds 500
}

if (-not $freshLogDetected) {
    Write-Host "Keil did not generate a fresh build log." -ForegroundColor Red
    Write-Host "Current log time: $($buildLogItem.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'))"
    Write-Host "The build log did not change within the wait window after UV4 returned." -ForegroundColor Yellow
    Write-Host "This can happen if UV4 is still holding the project, or if the log flush is delayed longer than expected." -ForegroundColor Yellow
    Write-Host "Try running the command once more, or use -ForceCloseUV4 if you want the script to own the whole build." -ForegroundColor Yellow
    exit 4
}

$logContent = $null
$logLines = @()
$resultLine = $null

for ($attempt = 0; $attempt -lt 20; $attempt++) {
    $logContent = Get-Content -LiteralPath $buildLog -Raw
    $logLines = Get-BuildLogLines -Content $logContent
    $resultLine = [regex]::Match($logContent, '"\.\\objects\\AT32F403ACCT7_WorkBench\.axf" - (\d+) Error\(s\), (\d+) Warning\(s\)\.')

    if ($resultLine.Success) {
        break
    }

    Start-Sleep -Milliseconds 500
}

Write-FullBuildLog -Lines $logLines

if (-not $resultLine.Success) {
    Write-Host "Build finished, but the summary line was not found in the HTML log." -ForegroundColor Yellow
    Write-Host "Log: $buildLog"
    exit 3
}

$errorCount = [int]$resultLine.Groups[1].Value
$warningCount = [int]$resultLine.Groups[2].Value

if ($errorCount -ne 0) {
    Write-Host "Build failed with $errorCount error(s) and $warningCount warning(s)." -ForegroundColor Red
    Write-BuildDiagnostics -Lines $logLines -Kind error
    if ($warningCount -gt 0) {
        Write-BuildDiagnostics -Lines $logLines -Kind warning
    }
    Write-Host "Log: $buildLog"
    exit 1
}

Write-Host "Build succeeded with $warningCount warning(s)." -ForegroundColor Green

if ($warningCount -gt 0) {
    Write-BuildDiagnostics -Lines $logLines -Kind warning
}

if (Test-Path -LiteralPath $axfFile) {
    $axf = Get-Item -LiteralPath $axfFile
    Write-Host "AXF: $($axf.FullName)"
    Write-Host "AXF Time: $($axf.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'))"
}

if (Test-Path -LiteralPath $hexFile) {
    $hex = Get-Item -LiteralPath $hexFile
    Write-Host "HEX: $($hex.FullName)"
    Write-Host "HEX Time: $($hex.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'))"
}

Write-Host "Log: $buildLog"
exit 0
