param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("baselines", "conditioning", "odometry_coverage",
                 "relay_noise", "communication", "odometry", "drift",
                 "yaw_step", "covariance_ablation")]
    [string]$Study,
    [int]$N = 200,
    [string]$Method = "-",
    [string]$Axis = "",
    [switch]$AllowDirty,
    [switch]$AllowOverwrite,
    [string]$BuildDir = "$env:TEMP\gps_free_seeking_campaign_build"
)

$ErrorActionPreference = "Stop"
$sourceDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$repoDir = (Resolve-Path (Join-Path $sourceDir "..")).Path
$head = (git -C $repoDir rev-parse HEAD).Trim()
$status = @(git -C $repoDir status --porcelain --untracked-files=no)
$dirty = $status.Count -gt 0

if ($dirty -and -not $AllowDirty) {
    throw "Tracked gps_free_seeking sources are dirty. Commit them or rerun with -AllowDirty for non-citable diagnostics."
}

$runId = if ($dirty) {
    "$head`_dirty_$([DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))"
} else {
    $head
}
$runDir = Join-Path $sourceDir "results\campaign2027\offline\run_$runId"
$methodSuffix = if ($Method -eq "-") { "all" } else { $Method }
$existingManifest = switch ($Study) {
    "baselines" { "s3_baselines_manifest_$methodSuffix.json" }
    "conditioning" { "s2_conditioning_manifest.json" }
    "odometry_coverage" { "s2_odometry_coverage_manifest.json" }
    "relay_noise" { "s4_relay_noise_manifest.json" }
    "communication" { "s8_communication_manifest.json" }
    "odometry" {
        $axisSuffix = if ($Axis) { "_$Axis" } else { "" }
        "s5_odometry_manifest_$methodSuffix$axisSuffix.json"
    }
    "drift" { "s6_drift_manifest.json" }
    "yaw_step" {
        $scenario = if ($Method -eq "-") { "transit" } else { $Method }
        "s7_disturbance_${scenario}_manifest.json"
    }
    "covariance_ablation" { "s9_covariance_ablation_manifest.json" }
}
if (-not $dirty -and -not $AllowOverwrite -and
    (Test-Path (Join-Path $runDir $existingManifest))) {
    throw "This clean study already exists for commit $head. Use -AllowOverwrite only for an intentional replacement."
}

cmake -S $sourceDir -B $BuildDir -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed." }
cmake --build $BuildDir --target gps_free_seeking_campaign --parallel
if ($LASTEXITCODE -ne 0) { throw "Campaign build failed." }

$env:GFS_GIT_DIRTY = if ($dirty) { "true" } else { "false" }
$env:GFS_RUN_ID = $runId
$exe = Join-Path $BuildDir "gps_free_seeking_campaign.exe"
$args = @($Study, $N, $Method, $head)
if ($Axis) { $args += $Axis }
Push-Location $sourceDir
try {
    & $exe @args
    if ($LASTEXITCODE -ne 0) {
        throw "Campaign failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

if (Test-Path $runDir) {
    $checksums = Get-ChildItem $runDir -File |
        Where-Object Name -ne "checksums.sha256" |
        Sort-Object Name |
        ForEach-Object {
            $hash = (Get-FileHash $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
            "$hash  $($_.Name)"
        }
    Set-Content -Path (Join-Path $runDir "checksums.sha256") -Value $checksums
}

Write-Host "Completed $Study in run_$runId (dirty=$dirty)."
