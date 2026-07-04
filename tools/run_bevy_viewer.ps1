param(
    [string]$UrdfPath = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf",
    [int]$TriangleCap = -1,
    [string]$MeshDir = "",
    [string]$Joints = "",
    [string]$TargetXyz = "",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

if (-not [System.IO.Path]::IsPathRooted($UrdfPath)) {
    $UrdfPath = Join-Path $RepoRoot $UrdfPath
}
if ($MeshDir -ne "" -and -not [System.IO.Path]::IsPathRooted($MeshDir)) {
    $MeshDir = Join-Path $RepoRoot $MeshDir
}

$viewerArgs = @()
if ($UrdfPath -ne "") {
    $viewerArgs += @("--urdf", $UrdfPath)
}
if ($TriangleCap -ge 0) {
    $viewerArgs += @("--tri-cap", $TriangleCap)
}
if ($MeshDir -ne "") {
    $viewerArgs += @("--mesh-dir", $MeshDir)
}
if ($Joints -ne "") {
    $viewerArgs += "--joints=$Joints"
}
if ($TargetXyz -ne "") {
    $viewerArgs += "--target-xyz=$TargetXyz"
}
if ($DryRun) {
    $viewerArgs += "--dry-run"
}

$manifestPath = Join-Path $PSScriptRoot "bevy_viewer\Cargo.toml"
cargo run --quiet --manifest-path $manifestPath -- @viewerArgs


