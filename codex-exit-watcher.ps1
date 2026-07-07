$ErrorActionPreference = "SilentlyContinue"

$pollIntervalMs = 2000
$missingWindowThreshold = 2

$windowSeen = $false
$missingWindowCount = 0

while ($true) {
    $codexProcesses = @(Get-Process | Where-Object { $_.ProcessName -ieq "Codex" })

    if ($codexProcesses.Count -eq 0) {
        $windowSeen = $false
        $missingWindowCount = 0
        Start-Sleep -Milliseconds $pollIntervalMs
        continue
    }

    $visibleWindowProcesses = @(
        $codexProcesses | Where-Object { $_.MainWindowHandle -ne 0 }
    )

    if ($visibleWindowProcesses.Count -gt 0) {
        $windowSeen = $true
        $missingWindowCount = 0
        Start-Sleep -Milliseconds $pollIntervalMs
        continue
    }

    if (-not $windowSeen) {
        Start-Sleep -Milliseconds $pollIntervalMs
        continue
    }

    $missingWindowCount += 1

    if ($missingWindowCount -lt $missingWindowThreshold) {
        Start-Sleep -Milliseconds $pollIntervalMs
        continue
    }

    $codexProcesses | Stop-Process -Force

    $windowSeen = $false
    $missingWindowCount = 0

    Start-Sleep -Milliseconds $pollIntervalMs
}
