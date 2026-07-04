param(
    [string]$UrdfPath = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf",
    [string]$Joints = "0,0,0,0,0,0",
    [string]$TargetLink = "link6",
    [string]$TargetXyz = "",
    [string]$TargetRpy = "",
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
    if ($TargetRpy -ne "") {
        $argsList += "--target-rpy=$TargetRpy"
    }
}

cargo run --quiet --manifest-path $manifestPath -- @argsList
