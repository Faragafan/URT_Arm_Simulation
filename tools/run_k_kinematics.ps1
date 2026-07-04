param(
    [string]$UrdfPath = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf",
    [string]$Joints = "0,0,0,0,0",
    [string]$TargetLink = "link5",
    [string]$TargetXyz = "",
    [switch]$List
)

$ErrorActionPreference = "Stop"
$manifestPath = Join-Path $PSScriptRoot "k_kinematics\Cargo.toml"
$argsList = @("--urdf", $UrdfPath, "--target-link", $TargetLink)

if ($List) {
    $argsList += "--list"
} else {
    $argsList += "--joints=$Joints"
    if ($TargetXyz -ne "") {
        $argsList += "--target-xyz=$TargetXyz"
    }
}

cargo run --quiet --manifest-path $manifestPath -- @argsList

