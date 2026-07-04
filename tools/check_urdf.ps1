param(
    [string]$UrdfPath = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

if (-not [System.IO.Path]::IsPathRooted($UrdfPath)) {
    $UrdfPath = Join-Path $RepoRoot $UrdfPath
}

if (-not (Test-Path -LiteralPath $UrdfPath)) {
    throw "URDF not found: $UrdfPath"
}

$urdfItem = Get-Item -LiteralPath $UrdfPath
$urdfDir = $urdfItem.Directory.FullName
[xml]$urdf = Get-Content -LiteralPath $urdfItem.FullName

$links = @($urdf.robot.link)
$joints = @($urdf.robot.joint)
$linkNames = @($links | ForEach-Object { $_.name })
$parentNames = @($joints | ForEach-Object { $_.parent.link })
$childNames = @($joints | ForEach-Object { $_.child.link })

Write-Host "URDF: $($urdfItem.FullName)"
Write-Host "Robot: $($urdf.robot.name)"
Write-Host "Links: $($links.Count)"
Write-Host "Joints: $($joints.Count)"
Write-Host ""

$errors = New-Object System.Collections.Generic.List[string]

foreach ($joint in $joints) {
    if ($linkNames -notcontains $joint.parent.link) {
        $errors.Add("Joint '$($joint.name)' parent link missing: $($joint.parent.link)")
    }
    if ($linkNames -notcontains $joint.child.link) {
        $errors.Add("Joint '$($joint.name)' child link missing: $($joint.child.link)")
    }
    if (-not $joint.origin -or -not $joint.origin.xyz -or -not $joint.origin.rpy) {
        $errors.Add("Joint '$($joint.name)' missing origin xyz/rpy")
    }
    if (($joint.type -eq "revolute" -or $joint.type -eq "continuous" -or $joint.type -eq "prismatic") -and -not $joint.axis) {
        $errors.Add("Joint '$($joint.name)' missing axis")
    }
}

$rootLinks = @($linkNames | Where-Object { $childNames -notcontains $_ })
if ($rootLinks.Count -ne 1) {
    $errors.Add("Expected exactly one root link, found $($rootLinks.Count): $($rootLinks -join ', ')")
}

foreach ($mesh in $urdf.SelectNodes("//mesh")) {
    $meshPath = Join-Path $urdfDir $mesh.filename
    if (-not (Test-Path -LiteralPath $meshPath)) {
        $errors.Add("Mesh missing: $($mesh.filename)")
    }
}

Write-Host "Joint chain:"
foreach ($joint in $joints) {
    $origin = if ($joint.origin) { $joint.origin.xyz } else { "" }
    $axis = if ($joint.axis) { $joint.axis.xyz } else { "" }
    Write-Host ("  {0}: {1} -> {2}, axis=[{3}], origin=[{4}]" -f $joint.name, $joint.parent.link, $joint.child.link, $axis, $origin)
}
Write-Host ""

if ($errors.Count -gt 0) {
    Write-Host "Errors:" -ForegroundColor Red
    foreach ($errorMessage in $errors) {
        Write-Host "  - $errorMessage" -ForegroundColor Red
    }
    exit 1
}

Write-Host "URDF checks passed." -ForegroundColor Green
