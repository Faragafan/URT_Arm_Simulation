use anyhow::{bail, Context, Result};
use clap::Parser;
use k::InverseKinematicsSolver;
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[command(about = "Forward and inverse kinematics demo using the k crate and the URT arm URDF")]
struct Args {
    #[arg(long, default_value = "assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf")]
    urdf: PathBuf,

    #[arg(long, value_delimiter = ',', num_args = 0..)]
    joints: Vec<f64>,

    #[arg(long, default_value = "link5")]
    target_link: String,

    #[arg(long, value_delimiter = ',')]
    target_xyz: Option<Vec<f64>>,

    #[arg(long)]
    list: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .context("failed to resolve repo root from Cargo manifest path")?
        .to_path_buf();

    let urdf_path = if args.urdf.is_absolute() {
        args.urdf
    } else {
        repo_root.join(args.urdf)
    };

    let chain = k::Chain::<f64>::from_urdf_file(&urdf_path)
        .with_context(|| format!("failed to load URDF {}", urdf_path.display()))?;

    let joint_names = chain
        .iter_joints()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();

    println!("URDF: {}", urdf_path.display());
    println!("DOF: {}", chain.dof());
    println!("Movable joints:");
    for (index, name) in joint_names.iter().enumerate() {
        println!("  [{index}] {name}");
    }

    if args.list {
        println!("\nAll nodes:");
        for node in chain.iter() {
            println!("  {}", node.joint().name);
        }
        return Ok(());
    }

    let positions = if args.joints.is_empty() {
        vec![0.0; chain.dof()]
    } else {
        args.joints
    };

    if positions.len() != chain.dof() {
        bail!(
            "expected {} joint values, got {}. Example: --joints 0,0.2,-0.4,0,0.1",
            chain.dof(),
            positions.len()
        );
    }

    chain
        .set_joint_positions(&positions)
        .context("failed to set joint positions")?;
    chain.update_transforms();

    if let Some(target_xyz) = args.target_xyz.as_ref() {
        if target_xyz.len() != 3 {
            bail!(
                "expected --target-xyz to have 3 comma-separated values, got {}",
                target_xyz.len()
            );
        }
        solve_ik(&chain, &args.target_link, target_xyz)?;
    }

    println!("\nJoint positions:");
    for (name, value) in joint_names.iter().zip(chain.joint_positions().iter()) {
        println!("  {name}: {value:.6} rad");
    }

    println!("\nWorld poses by node:");
    for node in chain.iter() {
        if let Some(transform) = node.world_transform() {
            print_pose(&node.joint().name, &transform);
        }
    }

    println!("\nTarget link pose:");
    let target = chain
        .find_link(&args.target_link)
        .with_context(|| format!("target link '{}' was not found in the URDF", args.target_link))?;
    let target_transform = target
        .world_transform()
        .with_context(|| format!("target link '{}' has no world transform", args.target_link))?;
    print_pose(&args.target_link, &target_transform);

    Ok(())
}

fn solve_ik(chain: &k::Chain<f64>, target_link: &str, target_xyz: &[f64]) -> Result<()> {
    let target_node = chain
        .find_link(target_link)
        .with_context(|| format!("target link '{target_link}' was not found in the URDF"))?;
    let mut target_transform = target_node
        .world_transform()
        .with_context(|| format!("target link '{target_link}' has no world transform"))?;

    target_transform.translation.vector.x = target_xyz[0];
    target_transform.translation.vector.y = target_xyz[1];
    target_transform.translation.vector.z = target_xyz[2];

    let arm = k::SerialChain::from_end(target_node);
    let solver = k::JacobianIkSolver::default();
    let mut constraints = k::Constraints::default();
    constraints.rotation_x = false;
    constraints.rotation_y = false;
    constraints.rotation_z = false;

    solver
        .solve_with_constraints(&arm, &target_transform, &constraints)
        .context("position-only IK solver failed")?;
    chain.update_transforms();

    println!(
        "\nIK requested target for {target_link}: xyz=[{:.6}, {:.6}, {:.6}]",
        target_xyz[0], target_xyz[1], target_xyz[2]
    );

    Ok(())
}

fn print_pose(label: &str, transform: &k::Isometry3<f64>) {
    let t = transform.translation.vector;
    let q = transform.rotation.quaternion();
    println!(
        "  {label:<16} xyz=[{:.6}, {:.6}, {:.6}] quat_xyzw=[{:.6}, {:.6}, {:.6}, {:.6}]",
        t.x, t.y, t.z, q.i, q.j, q.k, q.w
    );
}

