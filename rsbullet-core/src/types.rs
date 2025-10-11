use std::path::PathBuf;

use nalgebra as na;
use rsbullet_sys as ffi;

#[derive(Debug)]
pub(crate) struct CommandStatus {
    pub(crate) handle: ffi::b3SharedMemoryStatusHandle,
    pub(crate) status_type: i32,
}

#[derive(Debug, Clone)]
pub struct BodyInfo {
    pub base_name: String,
    pub body_name: String,
}

#[derive(Debug, Clone, Default)]
pub struct UrdfOptions {
    pub base: Option<na::Isometry3<f64>>,
    pub use_maximal_coordinates: Option<bool>,
    pub use_fixed_base: bool,
    pub flags: i32,
    pub global_scaling: Option<f64>,
}

impl From<na::Isometry3<f64>> for UrdfOptions {
    fn from(base: na::Isometry3<f64>) -> Self {
        Self {
            base: Some(base),
            ..Self::default()
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct DynamicsInfo {
    pub mass: f64,
    pub lateral_friction: f64,
    pub local_inertia_diagonal: [f64; 3],
    pub local_inertia_position: [f64; 3],
    pub local_inertia_orientation: [f64; 4],
    pub restitution: f64,
    pub rolling_friction: f64,
    pub spinning_friction: f64,
    pub contact_damping: f64,
    pub contact_stiffness: f64,
    pub body_type: i32,
    pub collision_margin: f64,
    pub angular_damping: f64,
    pub linear_damping: f64,
    pub ccd_swept_sphere_radius: f64,
    pub contact_processing_threshold: f64,
    pub activation_state: i32,
    pub friction_anchor: bool,
    pub dynamic_type: i32,
}

#[allow(dead_code)]
#[derive(Debug, Default, Clone)]
pub struct DynamicsUpdate {
    pub mass: Option<f64>,
    pub local_inertia_diagonal: Option<[f64; 3]>,
    pub anisotropic_friction: Option<[f64; 3]>,
    pub joint_limit: Option<(f64, f64)>,
    pub joint_limit_force: Option<f64>,
    pub dynamic_type: Option<i32>,
    pub lateral_friction: Option<f64>,
    pub spinning_friction: Option<f64>,
    pub rolling_friction: Option<f64>,
    pub restitution: Option<f64>,
    pub linear_damping: Option<f64>,
    pub angular_damping: Option<f64>,
    pub joint_damping: Option<f64>,
    pub contact_stiffness_and_damping: Option<(f64, f64)>,
    pub friction_anchor: Option<bool>,
    pub ccd_swept_sphere_radius: Option<f64>,
    pub contact_processing_threshold: Option<f64>,
    pub activation_state: Option<i32>,
    pub max_joint_velocity: Option<f64>,
    pub collision_margin: Option<f64>,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum ForceFrame {
    Link,
    World,
}

impl ForceFrame {
    #[allow(dead_code)]
    pub fn as_raw(self) -> i32 {
        match self {
            ForceFrame::Link => 1,
            ForceFrame::World => 2,
        }
    }
}
#[derive(Debug, Clone, Default)]
pub struct SdfOptions {
    pub use_maximal_coordinates: Option<bool>,
    pub global_scaling: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct MjcfOptions {
    pub flags: Option<i32>,
    pub use_multi_body: Option<bool>,
}

#[derive(Debug, Clone, Copy)]
pub struct BaseState {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

#[derive(Debug, Clone, Copy)]
pub struct BaseVelocity {
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct JointInfo {
    pub joint_index: i32,
    pub joint_name: String,
    pub joint_type: i32,
    pub q_index: i32,
    pub u_index: i32,
    pub q_size: i32,
    pub u_size: i32,
    pub flags: i32,
    pub damping: f64,
    pub friction: f64,
    pub lower_limit: f64,
    pub upper_limit: f64,
    pub max_force: f64,
    pub max_velocity: f64,
    pub link_name: String,
    pub joint_axis: [f64; 3],
    pub parent_frame_pos: [f64; 3],
    pub parent_frame_orn: [f64; 4],
    pub parent_index: i32,
}

#[derive(Debug, Clone)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
    pub force_torque: [f64; 6],
    pub motor_torque: f64,
}

/// Options for restoring a saved simulation state.
#[derive(Debug, Clone, Default)]
pub struct RestoreStateOptions {
    pub state_id: Option<i32>,
    pub file: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UserDataLookupOptions {
    pub link_index: Option<i32>,
    pub visual_shape_index: Option<i32>,
}

#[derive(Debug, Clone)]
pub struct UserDataInfo {
    pub user_data_id: i32,
    pub key: String,
    pub body_unique_id: i32,
    pub link_index: i32,
    pub visual_shape_index: i32,
}

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct LinkState {
    pub world_position: [f64; 3],
    pub world_orientation: [f64; 4],
    pub local_inertial_position: [f64; 3],
    pub local_inertial_orientation: [f64; 4],
    pub world_link_frame_position: [f64; 3],
    pub world_link_frame_orientation: [f64; 4],
    pub world_linear_velocity: Option<[f64; 3]>,
    pub world_angular_velocity: Option<[f64; 3]>,
    pub world_aabb: Option<Aabb>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LinkStateOptions {
    pub compute_forward_kinematics: bool,
    pub compute_link_velocity: bool,
}

#[derive(Debug, Clone)]
pub struct JointStateMultiDof {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub reaction_forces: [f64; 6],
    pub motor_torques: Vec<f64>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MultiDofTarget<'a> {
    pub joint_index: i32,
    pub positions: Option<&'a [f64]>,
    pub velocities: Option<&'a [f64]>,
}

// =================== Simulation Parameters Types ===================

/// Snapshot of current physics engine parameters (mirrors ffi::b3PhysicsSimulationParameters)
#[derive(Debug, Clone, Default)]
pub struct PhysicsSimulationParameters {
    pub delta_time: f64,
    pub simulation_timestamp: f64,
    pub gravity_acceleration: [f64; 3],
    pub num_simulation_sub_steps: i32,
    pub num_solver_iterations: i32,
    pub warm_starting_factor: f64,
    pub articulated_warm_starting_factor: f64,
    pub use_real_time_simulation: bool,
    pub use_split_impulse: bool,
    pub split_impulse_penetration_threshold: f64,
    pub contact_breaking_threshold: f64,
    pub internal_sim_flags: i32,
    pub default_contact_erp: f64,
    pub collision_filter_mode: i32,
    pub enable_file_caching: bool,
    pub restitution_velocity_threshold: f64,
    pub default_non_contact_erp: f64,
    pub friction_erp: f64,
    pub default_global_cfm: f64,
    pub friction_cfm: f64,
    pub enable_cone_friction: bool,
    pub deterministic_overlapping_pairs: bool,
    pub allowed_ccd_penetration: f64,
    pub joint_feedback_mode: i32,
    pub solver_residual_threshold: f64,
    pub contact_slop: f64,
    pub enable_sat: bool,
    pub constraint_solver_type: i32,
    pub minimum_solver_island_size: i32,
    pub report_solver_analytics: bool,
    pub sparse_sdf_voxel_size: f64,
    pub num_non_contact_inner_iterations: i32,
}

/// Partial update for physics engine parameters. Use None to skip a field.
#[derive(Debug, Clone, Default)]
pub struct PhysicsEngineParametersUpdate {
    pub default_contact_erp: Option<f64>,
    pub default_non_contact_erp: Option<f64>,
    pub friction_erp: Option<f64>,
    pub default_global_cfm: Option<f64>,
    pub friction_cfm: Option<f64>,
    pub num_sub_steps: Option<i32>,
    pub num_solver_iterations: Option<i32>,
    pub warm_starting_factor: Option<f64>,
    pub articulated_warm_starting_factor: Option<f64>,
    pub collision_filter_mode: Option<i32>,
    pub use_split_impulse: Option<bool>,
    pub split_impulse_penetration_threshold: Option<f64>,
    pub contact_breaking_threshold: Option<f64>,
    pub max_num_commands_per_1ms: Option<i32>,
    pub enable_file_caching: Option<bool>,
    pub restitution_velocity_threshold: Option<f64>,
    pub enable_cone_friction: Option<bool>,
    pub deterministic_overlapping_pairs: Option<bool>,
    pub allowed_ccd_penetration: Option<f64>,
    pub joint_feedback_mode: Option<i32>,
    pub solver_residual_threshold: Option<f64>,
    pub contact_slop: Option<f64>,
    pub enable_sat: Option<bool>,
    pub constraint_solver_type: Option<i32>,
    pub minimum_solver_island_size: Option<i32>,
    pub report_solver_analytics: Option<bool>,
    pub sparse_sdf_voxel_size: Option<f64>,
    pub num_non_contact_inner_iterations: Option<i32>,
    pub internal_sim_flags: Option<i32>,
}

// =================== Dynamics & Control Types ===================

#[derive(Debug, Clone, Copy, Default)]
pub enum JointControlMode {
    Velocity = 0,
    Torque = 1,
    #[default]
    PositionVelocityPd = 2,
    Pd = 3,
    StablePd = 4,
}

impl JointControlMode {
    pub fn as_raw(self) -> i32 {
        self as i32
    }

    pub fn is_position_based(self) -> bool {
        matches!(
            self,
            JointControlMode::PositionVelocityPd
                | JointControlMode::Pd
                | JointControlMode::StablePd
        )
    }

    pub fn is_velocity_based(self) -> bool {
        matches!(self, JointControlMode::Velocity)
    }

    pub fn is_torque_based(self) -> bool {
        matches!(self, JointControlMode::Torque)
    }
}

#[derive(Debug, Clone, Default)]
pub struct JointMotorControl2Options {
    pub joint_index: i32,
    pub control_mode: JointControlMode,
    pub target_position: Option<f64>,
    pub target_velocity: Option<f64>,
    pub force: Option<f64>,
    pub position_gain: Option<f64>,
    pub velocity_gain: Option<f64>,
    pub max_velocity: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct JointMotorControlArrayOptions<'a> {
    pub joint_indices: &'a [i32],
    pub control_mode: JointControlMode,
    pub target_positions: Option<&'a [f64]>,
    pub target_velocities: Option<&'a [f64]>,
    pub forces: Option<&'a [f64]>,
    pub position_gains: Option<&'a [f64]>,
    pub velocity_gains: Option<&'a [f64]>,
    pub max_velocity: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct JointMotorControlMultiDofOptions<'a> {
    pub joint_index: i32,
    pub control_mode: JointControlMode,
    pub target_positions: Option<&'a [f64]>,
    pub target_velocities: Option<&'a [f64]>,
    pub forces: Option<&'a [f64]>,
    pub position_gains: Option<&'a [f64]>,
    pub velocity_gains: Option<&'a [f64]>,
    pub damping: Option<&'a [f64]>,
    pub max_velocity: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct JointMotorControlMultiDofArrayEntry<'a> {
    pub joint_index: i32,
    pub target_positions: Option<&'a [f64]>,
    pub target_velocities: Option<&'a [f64]>,
    pub forces: Option<&'a [f64]>,
    pub position_gains: Option<&'a [f64]>,
    pub velocity_gains: Option<&'a [f64]>,
    pub damping: Option<&'a [f64]>,
    pub max_velocity: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct JointMotorControlMultiDofArrayOptions<'a> {
    pub control_mode: JointControlMode,
    pub entries: &'a [JointMotorControlMultiDofArrayEntry<'a>],
}

#[derive(Debug, Clone)]
pub struct Jacobian {
    pub linear: na::Matrix3xX<f64>,
    pub angular: na::Matrix3xX<f64>,
}

#[derive(Debug, Clone)]
pub struct MassMatrix {
    pub matrix: na::DMatrix<f64>,
}

impl Jacobian {
    pub fn zeros(columns: usize) -> Self {
        Self {
            linear: na::Matrix3xX::zeros(columns),
            angular: na::Matrix3xX::zeros(columns),
        }
    }
}

impl MassMatrix {
    pub fn as_matrix(&self) -> &na::DMatrix<f64> {
        &self.matrix
    }
}

impl std::ops::Deref for MassMatrix {
    type Target = na::DMatrix<f64>;

    fn deref(&self) -> &Self::Target {
        &self.matrix
    }
}

impl std::ops::DerefMut for MassMatrix {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.matrix
    }
}

#[derive(Debug, Clone, Default)]
pub struct InverseKinematicsOptions<'a> {
    pub target_orientation: Option<[f64; 4]>,
    pub lower_limits: Option<&'a [f64]>,
    pub upper_limits: Option<&'a [f64]>,
    pub joint_ranges: Option<&'a [f64]>,
    pub rest_poses: Option<&'a [f64]>,
    pub joint_damping: Option<&'a [f64]>,
    pub current_positions: Option<&'a [f64]>,
    pub solver: Option<i32>,
    pub max_iterations: Option<i32>,
    pub residual_threshold: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct InverseKinematicsMultiTargetOptions<'a> {
    pub end_effector_link_indices: &'a [i32],
    pub target_positions: &'a [[f64; 3]],
    pub lower_limits: Option<&'a [f64]>,
    pub upper_limits: Option<&'a [f64]>,
    pub joint_ranges: Option<&'a [f64]>,
    pub rest_poses: Option<&'a [f64]>,
    pub joint_damping: Option<&'a [f64]>,
    pub current_positions: Option<&'a [f64]>,
    pub solver: Option<i32>,
    pub max_iterations: Option<i32>,
    pub residual_threshold: Option<f64>,
}

// =================== Collision Query Types ===================

#[derive(Debug, Clone)]
pub struct ContactPoint {
    pub contact_flags: i32,
    pub body_a: i32,
    pub body_b: i32,
    pub link_a: i32,
    pub link_b: i32,
    pub position_on_a: [f64; 3],
    pub position_on_b: [f64; 3],
    pub contact_normal_on_b: [f64; 3],
    pub contact_distance: f64,
    pub normal_force: f64,
    pub linear_friction_force_1: f64,
    pub linear_friction_direction_1: [f64; 3],
    pub linear_friction_force_2: f64,
    pub linear_friction_direction_2: [f64; 3],
}

pub type ClosestPoint = ContactPoint;

#[derive(Debug, Clone, Copy)]
pub struct OverlappingObject {
    pub object_unique_id: i32,
    pub link_index: i32,
}

#[derive(Debug, Clone)]
pub struct RayHit {
    pub object_unique_id: i32,
    pub link_index: i32,
    pub hit_fraction: f64,
    pub hit_position_world: [f64; 3],
    pub hit_normal_world: [f64; 3],
}

#[derive(Debug, Clone, Default)]
pub struct ClosestPointsOptions {
    pub link_index_a: Option<i32>,
    pub link_index_b: Option<i32>,
    pub collision_shape_a: Option<i32>,
    pub collision_shape_b: Option<i32>,
    pub collision_shape_position_a: Option<[f64; 3]>,
    pub collision_shape_position_b: Option<[f64; 3]>,
    pub collision_shape_orientation_a: Option<[f64; 4]>,
    pub collision_shape_orientation_b: Option<[f64; 4]>,
}

#[derive(Debug, Clone)]
pub struct RayTestBatchOptions<'a> {
    pub num_threads: Option<i32>,
    pub parent_object_unique_id: Option<i32>,
    pub parent_link_index: Option<i32>,
    pub report_hit_number: Option<i32>,
    pub collision_filter_mask: Option<i32>,
    pub fraction_epsilon: Option<f64>,
    pub ray_from_positions: &'a [[f64; 3]],
    pub ray_to_positions: &'a [[f64; 3]],
}

// =================== Asset Creation & Mutation Types ===================

/// Build an `Isometry3` from position array and orientation quaternion in XYZW order.
pub fn isometry_from_raw_parts(
    position: [f64; 3],
    orientation_xyzw: [f64; 4],
) -> na::Isometry3<f64> {
    let translation = na::Translation3::new(position[0], position[1], position[2]);
    let quaternion = na::Quaternion::new(
        orientation_xyzw[3],
        orientation_xyzw[0],
        orientation_xyzw[1],
        orientation_xyzw[2],
    );
    let rotation = na::UnitQuaternion::from_quaternion(quaternion);
    na::Isometry3::from_parts(translation, rotation)
}

/// Build an `Isometry3` from a `[position, orientation]` frame slice.
pub fn isometry_from_frame(frame: &[f64; 7]) -> na::Isometry3<f64> {
    isometry_from_raw_parts(
        [frame[0], frame[1], frame[2]],
        [frame[3], frame[4], frame[5], frame[6]],
    )
}

/// Extract translation (XYZ) and quaternion (XYZW) components from an `Isometry3`.
pub fn isometry_to_raw_parts(transform: &na::Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let rotation = transform.rotation;
    (
        transform.translation.into(),
        [rotation.i, rotation.j, rotation.k, rotation.w],
    )
}

/// Write an `Isometry3` into a `[position, orientation]` frame buffer.
pub fn isometry_write_to_frame(transform: &na::Isometry3<f64>, frame: &mut [f64; 7]) {
    let (position, orientation) = isometry_to_raw_parts(transform);
    frame[..3].copy_from_slice(&position);
    frame[3..7].copy_from_slice(&orientation);
}

#[derive(Debug, Clone)]
pub enum CollisionGeometry<'a> {
    Sphere {
        radius: f64,
    },
    Box {
        half_extents: [f64; 3],
    },
    Capsule {
        radius: f64,
        height: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Plane {
        normal: [f64; 3],
        constant: f64,
    },
    Mesh {
        file: &'a str,
        scale: [f64; 3],
    },
    ConvexMesh {
        vertices: &'a [[f64; 3]],
        scale: [f64; 3],
    },
    ConcaveMesh {
        vertices: &'a [[f64; 3]],
        indices: &'a [i32],
        scale: [f64; 3],
    },
    HeightfieldFile {
        file: &'a str,
        mesh_scale: [f64; 3],
        texture_scaling: f64,
    },
    HeightfieldData {
        mesh_scale: [f64; 3],
        texture_scaling: f64,
        samples: &'a [f32],
        rows: i32,
        columns: i32,
        replace_index: Option<i32>,
    },
}

#[derive(Debug, Clone)]
pub struct CollisionShapeOptions<'a> {
    pub geometry: CollisionGeometry<'a>,
    pub flags: Option<i32>,
    pub child_transform: Option<na::Isometry3<f64>>,
}

#[derive(Debug, Clone)]
pub struct CollisionShapeChild<'a> {
    pub geometry: CollisionGeometry<'a>,
    pub transform: Option<na::Isometry3<f64>>,
    pub flags: Option<i32>,
}

#[derive(Debug, Clone)]
pub struct CollisionShapeArrayOptions<'a> {
    pub children: &'a [CollisionShapeChild<'a>],
}

#[derive(Debug, Clone)]
pub enum VisualGeometry<'a> {
    Sphere {
        radius: f64,
    },
    Box {
        half_extents: [f64; 3],
    },
    Capsule {
        radius: f64,
        height: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Plane {
        normal: [f64; 3],
        constant: f64,
    },
    Mesh {
        file: &'a str,
        scale: [f64; 3],
    },
    MeshData {
        vertices: &'a [[f64; 3]],
        indices: Option<&'a [i32]>,
        normals: Option<&'a [[f64; 3]]>,
        uvs: Option<&'a [[f64; 2]]>,
        scale: [f64; 3],
    },
}

#[derive(Debug, Clone)]
pub struct VisualShapeOptions<'a> {
    pub geometry: VisualGeometry<'a>,
    pub rgba: Option<[f64; 4]>,
    pub specular: Option<[f64; 3]>,
    pub flags: Option<i32>,
    pub transform: Option<na::Isometry3<f64>>,
}

#[derive(Debug, Clone)]
pub struct VisualShapeArrayOptions<'a> {
    pub children: &'a [VisualShapeOptions<'a>],
}

#[derive(Debug, Clone)]
pub struct MultiBodyBase {
    pub mass: f64,
    pub collision_shape: i32,
    pub visual_shape: i32,
    pub world_transform: na::Isometry3<f64>,
    pub inertial_transform: na::Isometry3<f64>,
}

impl Default for MultiBodyBase {
    fn default() -> Self {
        Self {
            mass: 0.0,
            collision_shape: -1,
            visual_shape: -1,
            world_transform: na::Isometry3::identity(),
            inertial_transform: na::Isometry3::identity(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MultiBodyLink {
    pub mass: f64,
    pub collision_shape: i32,
    pub visual_shape: i32,
    pub parent_index: Option<usize>,
    pub joint_type: i32,
    pub joint_axis: [f64; 3],
    pub parent_transform: na::Isometry3<f64>,
    pub inertial_transform: na::Isometry3<f64>,
}

impl Default for MultiBodyLink {
    fn default() -> Self {
        Self {
            mass: 0.0,
            collision_shape: -1,
            visual_shape: -1,
            parent_index: None,
            joint_type: 0,
            joint_axis: [0.0; 3],
            parent_transform: na::Isometry3::identity(),
            inertial_transform: na::Isometry3::identity(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MultiBodyCreateOptions<'a> {
    pub base: MultiBodyBase,
    pub links: &'a [MultiBodyLink],
    pub use_maximal_coordinates: bool,
    pub flags: Option<i32>,
    pub batch_transforms: Option<&'a [na::Isometry3<f64>]>,
}

#[derive(Debug, Clone)]
pub struct ConstraintCreateOptions {
    pub parent_body: i32,
    pub parent_link: Option<usize>,
    pub child_body: i32,
    pub child_link: Option<usize>,
    pub joint_type: i32,
    pub parent_frame: na::Isometry3<f64>,
    pub child_frame: na::Isometry3<f64>,
    pub joint_axis: [f64; 3],
    pub max_applied_force: f64,
    pub gear_ratio: Option<f64>,
    pub gear_aux_link: Option<usize>,
    pub relative_position_target: Option<f64>,
    pub erp: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct ConstraintUpdate {
    pub child_frame: Option<na::Isometry3<f64>>,
    pub max_force: Option<f64>,
    pub gear_ratio: Option<f64>,
    pub gear_aux_link: Option<usize>,
    pub relative_position_target: Option<f64>,
    pub erp: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct ConstraintInfo {
    pub parent_body: i32,
    pub parent_link: i32,
    pub child_body: i32,
    pub child_link: i32,
    pub parent_frame: na::Isometry3<f64>,
    pub child_frame: na::Isometry3<f64>,
    pub joint_axis: [f64; 3],
    pub joint_type: i32,
    pub max_applied_force: f64,
    pub constraint_unique_id: i32,
    pub gear_ratio: f64,
    pub gear_aux_link: i32,
    pub relative_position_target: f64,
    pub erp: f64,
}

#[derive(Debug, Clone)]
pub struct ConstraintState {
    pub applied_forces: [f64; 6],
    pub dof_count: i32,
}

#[derive(Debug, Clone, Default)]
pub struct ChangeVisualShapeOptions {
    pub shape_index: Option<i32>,
    pub texture_unique_id: Option<i32>,
    pub rgba_color: Option<[f64; 4]>,
    pub specular_color: Option<[f64; 3]>,
    pub flags: Option<i32>,
}

#[derive(Debug, Clone)]
pub struct TextureData<'a> {
    pub width: i32,
    pub height: i32,
    pub rgb_pixels: &'a [u8],
}

#[derive(Debug, Clone)]
pub struct TextureInfo {
    pub texture_unique_id: i32,
}
