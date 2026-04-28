class_name Wheel
extends Node3D

@export_group("Skidmarks")
@export var skid_mark_emitter: GPUParticles3D
@export var skid_mark_sensitivity: float = 100.0
@export var leave_skid_marks: bool = true

@export_group("Required Nodes")
@export var shape_cast: ShapeCast3D
@export var spinning_meshes: Array[MeshInstance3D]
@export var fixed_meshes: Array[MeshInstance3D]
@export var spin_marker: Marker3D
@export var fixed_marker: Marker3D
@export var contact_point_marker: Marker3D
@export var lateral_force_arrow: Sprite3D
@export var longitudinal_force_arrow: Sprite3D
@export var suspension_force_arrow: Sprite3D
@export var patch_position_mesh: MeshInstance3D
@export var differential_lock_target: Wheel
@export var hub_marker: Marker3D

var car: Vehicle

@export_group("Suspension")
@export var suspension_spring_stiffness: float = 50.0
@export var suspension_dampening: float = 7.0
@export var suspension_rest_length: float = 0.15
@export var suspension_tire_stiffness: float = 120.0
@export var suspension_tire_damping: float = 7.0
@export var suspension_tire_height: float = 0.087

@export_group("WheelProperties")
@export var tire_model: TireModelGD
@export var camber_angle: float = 0.0
@export var use_steering: bool = false
@export var wheel_radius: float = 0.55
@export var tire_deformation_factor: float = 5.0
@export var tire_spin_resistance_rate: float = 15.0
@export var tire_friction_multiplier: float = 0.7
@export var tire_elasticity: float = 2000.0
@export var tire_damping_stiffness: float = 35.0

@export var abs_enabled: bool = true
@export var abs_sensitivity: float = 4.0

@export var drivetrain_influence: float = 1.0
@export var drivetrain_max_brake_torque: float = 30.0
@export var drivetrain_handbrake_bias: float = 30.0
@export var drivetrain_brake_bias: float = 0.8

@export_group("Diff")
@export var differential_locking_preload: float = 10.0
@export_range(0.0, 1.0) var differential_locking_power: float = 0.2
@export_range(0.0, 1.0) var differential_locking_coast: float = 0.2
@export var differential_viscosity: float = 0.0


# Fallback contact data for when shape_cast misses for a single tick
var last_contact_point: Vector3 = Vector3.ZERO
var last_contact_normal: Vector3 = Vector3.ZERO
var last_contact_ground: Ground = Ground.new()

var spin: float = 0.0

var hub_position: float = 0.0
var hub_previous_position: float = 0.0

var is_brake_locked: bool = false
var aligning_torque: float = 0.0
var distance_travelled: float = 0.0
var distance_travelled_all_wheel_drive: float = 0.0

var drivetrain_overdrive: float = 0.0
var drivetrain_torque: float = 0.0
var drivetrain_substantial_torque: float = 0.0
var drivetrain_braking: float = 0.0

# Converts spring compression units to Newtons
const SPRING_FORCE_NEWTON_FACTOR: float = 0.0169946645619466

var impulse: ImpulseData = ImpulseData.new()
var predicted_impulse: ImpulseData = ImpulseData.new()

var tire_patch_offset: Vector3 = Vector3.ZERO

var output_skidding: float = 0.0
var output_stressing: float = 0.0


func _ready() -> void:
	car = get_parent() as Vehicle
	shape_cast.add_exception(car)

	for mesh in spinning_meshes:
		remove_child(mesh)
		spin_marker.add_child(mesh)

	for mesh in fixed_meshes:
		remove_child(mesh)
		fixed_marker.add_child(mesh)

	# Mirror camber angle for wheels on the right side of the car
	var camber_z := -camber_angle if position.x > 0 else camber_angle
	hub_marker.rotation_degrees.z = camber_z


func _physics_process(delta: float) -> void:
	if car.physics_state == null:
		return

	contact_point_marker.visible = car.DB_forces_visible

	aligning_torque = 0.0
	var hertz_scale := delta * 60.0

	# Apply drivetrain torque to wheel spin
	spin += drivetrain_torque / (drivetrain_overdrive + 1.0)
	spin += drivetrain_substantial_torque
	aligning_torque += drivetrain_substantial_torque

	if drivetrain_braking > 0.0:
		var overdrive_adjusted_braking := drivetrain_braking / (drivetrain_overdrive + 1.0)
		var spin_reduction := spin / maxf(absf(spin) / overdrive_adjusted_braking, 1.0)
		spin -= spin_reduction
		aligning_torque -= spin / maxf(absf(spin) / overdrive_adjusted_braking, 1.0)

	shape_cast.target_position.x = - suspension_rest_length

	if is_brake_locked:
		spin_marker.rotate_x(spin * delta * 2.0)

	var contact_point: Vector3 = global_position + global_basis.orthonormalized() * shape_cast.target_position
	var contact_axis := Basis()

	var is_colliding := shape_cast.is_colliding()

	if is_colliding or last_contact_point != Vector3.ZERO:
		var collision_normal: Vector3
		var ground := Ground.new()

		if is_colliding:
			collision_normal = shape_cast.get_collision_normal(0)
			contact_point = shape_cast.get_collision_point(0)
			var collider := shape_cast.get_collider(0)
			if collider is Ground:
				ground = collider as Ground
			last_contact_normal = collision_normal
			last_contact_point = contact_point
			last_contact_ground = ground
		else:
			collision_normal = last_contact_normal
			contact_point = last_contact_point
			last_contact_normal = Vector3.ZERO
			last_contact_point = Vector3.ZERO
			last_contact_ground = Ground.new()

		# Ignore contacts above the wheel (e.g. ceilings)
		if (contact_point - global_position).dot(global_basis.y) > 0:
			return

		contact_axis = Basis(
			collision_normal.cross(global_basis.z),
			collision_normal,
			global_basis.x.cross(collision_normal)
		).orthonormalized()

		var world_offset: Vector3 = contact_point - car.global_position
		var tire_patch_global_velocity: Vector3 = car.physics_state.get_velocity_at_local_position(world_offset)
		var predicted_patch_global_velocity: Vector3 = car.predict_velocity_at_position(world_offset)

		var tire_patch_local_velocity: Vector3 = tire_patch_global_velocity * contact_axis.orthonormalized()
		var predicted_patch_local_velocity: Vector3 = predicted_patch_global_velocity * contact_axis.orthonormalized()

		# Snap contact visual to the ground plane (no lateral offset)
		contact_point_marker.global_position = contact_point
		contact_point_marker.position.x = 0.0
		contact_point = contact_point_marker.global_position
		contact_point_marker.global_basis = contact_axis

		hub_previous_position = hub_position

		var tire_stiffness_lateral: float = tire_elasticity
		var tire_damping_lateral: float = tire_damping_stiffness
		var tire_stiffness_longitudinal: float = tire_stiffness_lateral * tire_model.aspect_ratio
		var tire_damping_longitudinal: float = tire_damping_lateral * tire_model.aspect_ratio

		# Ratio of lateral slip to total wheel speed — blends lateral/longitudinal tire model
		var slip_angle_ratio: float = minf(
			absf(tire_patch_local_velocity.x) / (absf(spin * wheel_radius) + 1.0), 1.0)

		var suspension_damping_force: float = suspension_dampening * 1000.0
		var suspension_stiffness_force: float = suspension_spring_stiffness * SPRING_FORCE_NEWTON_FACTOR * 1000.0
		var tire_stiffness_force: float = maxf(suspension_tire_stiffness * SPRING_FORCE_NEWTON_FACTOR * 1000.0, 0.0001)
		var tire_damping_force: float = suspension_tire_damping * 1000.0

		var tire_compliance_ratio := 1.0
		var tire_damping_modifier := 1.0

		if tire_stiffness_force > 0.0 and suspension_stiffness_force > 0.0:
			tire_compliance_ratio = tire_stiffness_force / (suspension_stiffness_force + tire_stiffness_force)
			tire_damping_modifier = maxf(tire_damping_force / (suspension_damping_force + tire_damping_force), delta)

		var tire_compression_depth: float = wheel_radius / 2.0 - global_position.distance_to(contact_point)
		var hub_target_position: float = lerpf(suspension_rest_length, -tire_compression_depth, tire_compliance_ratio)
		hub_position = lerpf(hub_position, hub_target_position, tire_damping_modifier)

		var tire_sink_depth: float = maxf(tire_compression_depth + hub_position - suspension_tire_height, 0.0)
		hub_position -= tire_sink_depth
		hub_position = maxf(hub_position, -tire_compression_depth)

		var axle_velocity: float = hub_previous_position - hub_position
		var suspension_compression: float = maxf(suspension_rest_length - hub_position, 0.0)
		var spring_force: float = maxf(0.0, suspension_compression * suspension_stiffness_force + suspension_damping_force * axle_velocity)

		var axle_max_extension: float = suspension_rest_length
		var axle_position: float = lerpf(0.0, axle_max_extension, tire_compliance_ratio)
		var axle_tire_sink_amount: float = maxf(axle_max_extension - axle_position - suspension_tire_height, 0.0)
		axle_position += axle_tire_sink_amount

		var axle_suspension_compression: float = maxf(axle_position, 0.0)
		var axle_force_on_suspension: float = maxf(axle_suspension_compression * suspension_stiffness_force, 0.0)
		var axle_grip_reduction_factor: float = suspension_stiffness_force / tire_stiffness_force / suspension_tire_height
		var axle_max_grip: float = axle_force_on_suspension / (axle_tire_sink_amount * axle_grip_reduction_factor + 1.0)

		var vertical_approach_velocity: float = - predicted_patch_local_velocity.y

		var total_grip: float = minf(
			spring_force * maxf(1.0 + vertical_approach_velocity, 0.001),
			axle_max_grip
		) * tire_model.friction * tire_friction_multiplier * ground.grip

		var curved_grip_lateral: float = total_grip * (tire_model.peak_y * (1.0 - tire_model.linear) + tire_model.linear)
		var curved_grip_longitudinal: float = total_grip * (tire_model.peak_x * (1.0 - tire_model.linear) + tire_model.linear)

		var blended_tire_shape: float = lerpf(tire_model.shape_y, tire_model.shape_x, slip_angle_ratio)
		var blended_tire_peak: float = lerpf(tire_model.peak_y, tire_model.peak_x, slip_angle_ratio)
		var blended_peaked_grip: float = total_grip
		var blended_curved_grip: float = lerpf(curved_grip_lateral, curved_grip_longitudinal, slip_angle_ratio)

		# Difference between wheel surface speed and actual ground contact speed
		var wheel_speed_difference: float = spin - tire_patch_local_velocity.z / wheel_radius
		var predicted_lateral_velocity: float = tire_patch_local_velocity.x
		var predicted_slip_amount := 0.0

		is_brake_locked = drivetrain_braking > (total_grip * (tire_spin_resistance_rate * delta * (drivetrain_overdrive + 1.0)))

		if total_grip > 0.0:
			var spin_resistance_modifier: float = tire_spin_resistance_rate * delta / (drivetrain_overdrive + 1.0)
			predicted_slip_amount = maxf(
				Vector2(wheel_speed_difference, predicted_lateral_velocity).length() / spin_resistance_modifier / curved_grip_lateral - 1.0,
				0.0)
			var predicted_peaked_amount: float = 1.0 - (1.0 / (
				maxf(
					sqrt(pow(absf(wheel_speed_difference), 2.0) + pow(absf(predicted_lateral_velocity), 2.0))
					/ total_grip / spin_resistance_modifier - 1.0,
					0.0
				) + 1.0))

			if not is_brake_locked:
				var slip_correction_factor: float = predicted_slip_amount * (
					predicted_peaked_amount / tire_model.shape_y * tire_model.peak_y * (1.0 - tire_model.linear) + tire_model.linear
				) + 1.0
				spin -= wheel_speed_difference / slip_correction_factor
				aligning_torque -= wheel_speed_difference / slip_correction_factor

		var blended_tire_stiffness: float = lerpf(tire_stiffness_lateral, tire_stiffness_longitudinal, pow(slip_angle_ratio, 0.5))
		var blended_tire_damping: float = lerpf(tire_damping_lateral, tire_damping_longitudinal, pow(slip_angle_ratio, 0.5))

		var braking_force_limit: float = drivetrain_braking / (blended_tire_stiffness * delta) / tire_spin_resistance_rate
		var torque_force_limit: float = drivetrain_torque / (blended_tire_stiffness * delta) / tire_spin_resistance_rate

		tire_patch_offset.x -= tire_patch_global_velocity.x * delta
		tire_patch_offset.z -= tire_patch_global_velocity.z * delta
		tire_patch_offset -= contact_axis.y * (tire_patch_offset * contact_axis).y

		var tire_patch_displacement: Vector3 = tire_patch_offset * contact_axis

		var standstill_factor := 1.0
		if spring_force > 0.0:
			standstill_factor /= absf(spin * tire_deformation_factor / 1.5) / 40.0 + 1.0
		standstill_factor += vertical_approach_velocity / wheel_radius
		standstill_factor = clampf(standstill_factor, delta, 1.0)

		tire_patch_offset -= contact_axis[0] * tire_patch_displacement.x * (1.0 - standstill_factor)

		if not is_brake_locked:
			var longitudinal_clamp_excess: float = maxf(absf(tire_patch_displacement.z) - braking_force_limit, 0.0)
			if tire_patch_displacement.z < 0.0:
				tire_patch_offset += contact_axis[2] * longitudinal_clamp_excess
			else:
				tire_patch_offset -= contact_axis[2] * longitudinal_clamp_excess

		var patch_slip_amount := 0.0
		var patch_peaked_amount := 0.0

		if total_grip > 0.0:
			patch_slip_amount = maxf(tire_patch_offset.length() / (curved_grip_lateral / blended_tire_stiffness) - 1.0, 0.0)
			patch_peaked_amount = 1.0 - (1.0 / (
				maxf(tire_patch_offset.length() / (blended_peaked_grip / blended_tire_stiffness) - 1.0, 0.0) + 1.0))

		tire_patch_offset /= patch_slip_amount * (
			patch_peaked_amount / blended_tire_shape * blended_tire_peak * (1.0 - tire_model.linear) + tire_model.linear
		) + 1.0

		if not is_brake_locked:
			tire_patch_offset += contact_axis.z * torque_force_limit

		var tire_patch_displacement_global: Vector3 = tire_patch_offset * contact_axis

		var tire_force_vector := Vector3(
			tire_patch_local_velocity.x * blended_tire_damping * standstill_factor - tire_patch_displacement_global.x * blended_tire_stiffness,
			0.0,
			(tire_patch_local_velocity.z - spin * wheel_radius) * blended_tire_damping * standstill_factor - tire_patch_displacement_global.z * blended_tire_stiffness
		)

		if total_grip > 0.0 and is_brake_locked:
			spin -= wheel_speed_difference / (predicted_slip_amount + 1.0)
			aligning_torque -= wheel_speed_difference / (predicted_slip_amount + 1.0)

		var slip_amount := 0.0
		var peaked_amount := 0.0

		if total_grip > 0.0:
			slip_amount = maxf(tire_force_vector.length() / blended_curved_grip - 1.0, 0.0)
			peaked_amount = 1.0 - (1.0 / (
				maxf(tire_force_vector.length() / blended_peaked_grip - 1.0, 0.0) + 1.0))

			output_skidding = maxf(
				slip_amount * blended_curved_grip * 0.025 * total_grip
				+ patch_slip_amount * blended_curved_grip * 0.05 * total_grip,
				0.0)
			output_stressing = tire_force_vector.length() * minf(absf(spin), 1.0)

		tire_force_vector /= slip_amount * (
			peaked_amount / blended_tire_shape * blended_tire_peak * (1.0 - tire_model.linear) + tire_model.linear
		) + 1.0

		if total_grip > 0.0:
			var is_slipping := slip_amount > 0.0
			var force_length := tire_force_vector.length()
			_update_debug_arrow(lateral_force_arrow, tire_force_vector.x, force_length, blended_curved_grip, is_slipping)
			_update_debug_arrow(longitudinal_force_arrow, tire_force_vector.z, force_length, blended_curved_grip, is_slipping)

		suspension_force_arrow.scale.y = spring_force / 40.0

		var contact_forces: Vector3 = contact_axis.y * spring_force
		var predicted_forces := Vector3.ZERO

		if total_grip > 0.0:
			contact_forces -= contact_axis.x * tire_force_vector.x
			contact_forces -= contact_axis.z * tire_force_vector.z
			predicted_forces -= contact_axis.z * tire_force_vector.z

		var impulse_point: Vector3 = contact_point - car.global_position

		predicted_impulse.set_data(impulse_point, predicted_forces * hertz_scale)
		impulse.set_data(impulse_point, contact_forces * hertz_scale)

		if leave_skid_marks:
			var material := skid_mark_emitter.process_material as ParticleProcessMaterial
			material.color = ground.skid_color
			skid_mark_emitter.emitting = output_skidding > skid_mark_sensitivity * ground.skid_mark_sensitivity
			skid_mark_emitter.global_position = contact_point + Vector3.UP * 0.005
			var skid_velocity := car.get_point_velocity(skid_mark_emitter.global_position).normalized()
			if skid_velocity.cross(Vector3.UP).length_squared() > 0.001:
				skid_mark_emitter.look_at(
				skid_mark_emitter.global_position + skid_velocity
				)

	else:
		skid_mark_emitter.emitting = false
		output_skidding = 0.0
		output_stressing = 0.0
		contact_point_marker.global_position = contact_point
		impulse.set_data(Vector3.ZERO, Vector3.ZERO)
		predicted_impulse.set_data(Vector3.ZERO, Vector3.ZERO)
		tire_patch_offset = Vector3.ZERO
		hub_position = suspension_rest_length

	patch_position_mesh.position.x = tire_patch_offset.x * contact_axis[0].x + tire_patch_offset.y * contact_axis[0].z
	patch_position_mesh.position.z = tire_patch_offset.x * contact_axis[2].x + tire_patch_offset.y * contact_axis[2].z

	hub_marker.position.y = - hub_position
	if not is_brake_locked:
		spin_marker.rotate_x(spin * delta * 2.0)

	distance_travelled += spin
	distance_travelled_all_wheel_drive += spin


func _update_debug_arrow(
	arrow: Sprite3D,
	force_component: float,
	force_length: float,
	curved_grip: float,
	is_slipping: bool
) -> void:
	arrow.scale.y = force_component / 40.0
	arrow.rotation_degrees.x = 91.0 if arrow.scale.y > 0.0 else 89.0

	if is_slipping:
		arrow.modulate = Color(1, 0, 0)
	else:
		var load_ratio: float = 1.0 + (force_length - curved_grip) / curved_grip
		arrow.modulate = Color(0, load_ratio, 1.0 - load_ratio)


class ImpulseData:
	var position: Vector3 = Vector3.ZERO
	var force: Vector3 = Vector3.ZERO

	func set_data(pos: Vector3, f: Vector3) -> void:
		position = pos
		force = f
