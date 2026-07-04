param(
    [string]$UrdfPath = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf",
    [string]$MeshDir = "",
    [switch]$MeshOnly,
    [switch]$BoxVisuals,
    [switch]$KeepCollision,
    [switch]$ScaleDebug
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

if (-not [System.IO.Path]::IsPathRooted($UrdfPath)) {
    $UrdfPath = Join-Path $RepoRoot $UrdfPath
}
if ($MeshDir -ne "" -and -not [System.IO.Path]::IsPathRooted($MeshDir)) {
    $MeshDir = Join-Path $RepoRoot $MeshDir
}

$pyLauncher = Get-Command py -ErrorAction SilentlyContinue
$pythonCommand = Get-Command python -ErrorAction SilentlyContinue

if ($pyLauncher) {
    $Python = $pyLauncher.Source
} elseif ($pythonCommand) {
    $Python = $pythonCommand.Source
} else {
    throw "Could not find Python. Install Python or make sure 'py' or 'python' is on PATH."
}

$viewerScript = Join-Path $PSScriptRoot "view_urdf_pybullet.py"
$viewerArgs = @($viewerScript, "--urdf", $UrdfPath)
if ($MeshOnly) {
    $viewerArgs += "--mesh-only"
}
if ($BoxVisuals) {
    $viewerArgs += "--box-visuals"
}
if ($KeepCollision) {
    $viewerArgs += "--keep-collision"
}
if ($ScaleDebug) {
    $viewerArgs += "--scale-debug"
}
if ($MeshDir -ne "") {
    $viewerArgs += @("--mesh-dir", $MeshDir)
}

& $Python @viewerArgs
