class_name Vehicle
extends RigidBody3D


@export_group("Debug")
@export var debug: Debug
@export var DB_forces_visible: bool = false

@export_group("Audio")
@export var engine_off: AudioStreamPlayer3D
@export var engine_on: AudioStreamPlayer3D

@export_group("Vehicle")
@export var controls: ControlSettingsGD
@export var engine_drive_force: float = 4.0

enum EngineDriveForceBehavior {WHEELS_SYNCED, WHEELS_UNSYNCED}
@export var engine_drive_force_behavior: EngineDriveForceBehavior = EngineDriveForceBehavior.WHEELS_UNSYNCED

@export var engine_rev_up_speed: float = 150.0
@export var engine_rev_down_speed: float = 50.0
@export var engine_rev_alignment_rate: float = 50.0
@export var engine_idle_rpm: float = 800.0
@export var engine_max_rpm: float = 7000.0
@export var engine_stall_prevention: float = 2.0
@export var engine_torque_curve: Curve
@export var engine_decline_curve: Curve
@export var gearbox_forward_gear_ratios: Array[float] = [3.25, 1.782, 1.172, 1.0, 0.759]
@export var gearbox_final_drive_ratio: float = 3.772
@export var gearbox_reverse_gear_ratios: Array[float] = [3.0]
@export var gearbox_clutch_needed: float = 0.25

@export var diff_central_locking_preload: float = 0.0
@export var diff_central_locking_power: float = 0.0
@export var diff_central_locking_coast: float = 0.0

@export var steering_pivot_node: Node3D
@export var center_of_mass_node: Marker3D
@export var steering_radius: float = 4.0


# How fast the engine is spinning based on wheel speed through the drivetrain (RPM)
var wheel_driven_rpm: float = 0.0
# Current engine RPM
var rpm: float = 0.0
# Engine RPM from the previous physics frame
var previous_rpm: float = 0.0

var wheels: Array[Wheel] = []
var steering_wheels: Array[Wheel] = []
var wheel_count: int = 0
var gear: int = 0

# Total drivetrain weight (sum of drivetrain_influence across all driven wheels), cached from last frame
var cached_drivetrain_weight: float = 0.0

# The Z position of the steering pivot point (rear axle reference for Ackermann steering)
var steering_pivot_z: float = -1.151

# Current throttle position (0 = closed, 1 = wide open)
var throttle: float = 0.0

var physics_state: PhysicsDirectBodyState3D

# Raw steering input from the player (-1 = full left, 1 = full right)
var steering_input: float = 0.0
# Final steering angle after assist and speed scaling is applied
var steering_output: float = 0.0
# Smoothed accelerator pedal position (0–1)
var analog_accelerate: float = 0.0
# Smoothed brake pedal position (0–1)
var analog_decelerate: float = 0.0
# Smoothed handbrake position (0–1)
var analog_handbrake: float = 0.0
# Smoothed clutch pedal position (0–1)
var analog_clutching: float = 0.0

# Estimated maximum steering angle, used to scale steering assistance
var max_steer_estimate: float = 0.0
# Scales how strongly steering assistance corrects the vehicle heading
var steering_assistance_factor: float = 0.0
# Average radius of all driven wheels, weighted by drivetrain influence
var driven_wheel_radius: float = 0.0

# Vehicle velocity expressed in local (body) space
var local_velocity: Vector3 = Vector3.ZERO
# Angular velocity expressed in local (body) space
var local_angular_velocity: Vector3 = Vector3.ZERO

# Countdown timer: automatic clutch stays engaged while this is above zero during a gear shift
var auto_clutch_engage_timer: float = 0.0
# Countdown timer: throttle is blipped/cut while this is above zero during a gear shift
var auto_throttle_blip_timer: float = 0.0

# The gear the driver has requested; actual gear change happens once clutch conditions are met
var target_gear: int = 0
# True when the vehicle is currently driving in reverse
var on_reverse: bool = false

# Predicted linear velocity delta from pre-applied impulses this frame
var predicted_linear_velocity: Vector3 = Vector3.ZERO
# Predicted angular velocity delta from pre-applied impulses this frame
var predicted_angular_velocity: Vector3 = Vector3.ZERO

# The gear ratio from one gear below the current gear (used to compare RPM across shifts)
var previous_gear_ratio: float = 0.0

var torque_data: TorqueData = TorqueData.new()


# Returns the predicted velocity at a world-space position,
# accounting for impulses already applied this frame.
func predict_velocity_at_position(world_position: Vector3) -> Vector3:
	world_position += physics_state.transform.origin
	return predicted_linear_velocity + predicted_angular_velocity.cross(world_position - to_global(center_of_mass))


# Accumulates a future impulse so other systems (e.g. wheels) can read the predicted post-impulse velocity.
func predict_impulse(application_point: Vector3, impulse: Vector3) -> void:
	predicted_linear_velocity += impulse * physics_state.inverse_mass
	predicted_angular_velocity += get_inverse_inertia_tensor() * (application_point - center_of_mass * global_transform.basis).cross(impulse)


# Returns the velocity of a point on the rigid body due to both linear and rotational motion.
func get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)


func _ready() -> void:
	_prepare_wheels()
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = center_of_mass_node.position
	steering_pivot_z = steering_pivot_node.position.z


func _prepare_wheels() -> void:
	for node in get_children():
		if node is Wheel:
			var wheel := node as Wheel
			wheel_count += 1
			wheels.append(wheel)
			if wheel.use_steering:
				steering_wheels.append(wheel)


func _input(event: InputEvent) -> void:
	if event.is_action_pressed("gearup"):
		shift_gear(1)
	elif event.is_action_pressed("geardown"):
		shift_gear(-1)

	if event.is_action_pressed("debug_key"):
		DB_forces_visible = !DB_forces_visible


# Shifts the gearbox up (direction = 1) or down (direction = -1) by one gear.
func shift_gear(direction: int) -> void:
	var gear_limit := gearbox_forward_gear_ratios.size() if direction > 0 else -gearbox_reverse_gear_ratios.size()
	var within_range := target_gear < gear_limit if direction > 0 else target_gear > gear_limit
	if not within_range:
		return

	if controls.AST_shift_assistance > 0:
		# Automatic shift assistance: trigger clutch and throttle blip timers, then shift immediately.
		if target_gear != 0:
			auto_clutch_engage_timer = controls.AST_shifting_clutch_out_time
			auto_throttle_blip_timer = controls.AST_shifting_off_throttle_time
		target_gear += direction
	elif analog_clutching > gearbox_clutch_needed:
		# Manual: only shift if the driver has pressed the clutch far enough.
		target_gear += direction


# Applies the current gear selection and returns the combined gear + final drive ratio.
func gearbox() -> float:
	var gear_ratio := 1.0

	if controls.AST_shift_assistance > 0:
		# Commit the target gear once the clutch is engaged or we are in neutral.
		if analog_clutching > gearbox_clutch_needed or gear == 0:
			gear = target_gear
			if gear == 0:
				auto_clutch_engage_timer = -1
				auto_throttle_blip_timer = -1
		if auto_throttle_blip_timer <= 0:
			on_reverse = gear < 0
	else:
		gear = target_gear

	# Look up the ratio one gear below current, for cross-gear RPM comparisons.
	var previous_gear_index := absi(gear) - 2
	previous_gear_ratio = 0.001

	if gear > 0 and previous_gear_index >= 0:
		previous_gear_ratio = gearbox_forward_gear_ratios[previous_gear_index]
	elif gear < 0 and previous_gear_index >= 0:
		previous_gear_ratio = gearbox_reverse_gear_ratios[previous_gear_index]

	if gear > 0:
		gear_ratio = gearbox_forward_gear_ratios[gear - 1]
	elif gear < 0:
		gear_ratio = gearbox_reverse_gear_ratios[absi(gear) - 1]

	previous_gear_ratio *= gearbox_final_drive_ratio * 2.0
	return gear_ratio * gearbox_final_drive_ratio


# Simulates the engine: updates throttle, advances RPM, and writes torque output to torque_data.
func handle_engine(delta: float, rev_up_rate: float, rev_down_rate: float) -> void:
	var physics_steps_per_minute := delta * 60.0
	throttle -= (throttle - analog_accelerate) * 0.5

	# The throttle position at which rev-up and rev-down forces exactly balance.
	var torque_balance_point := rev_down_rate / (rev_up_rate + rev_down_rate)
	# How far below idle the engine currently is (positive = RPM is below idle).
	var distance_below_idle := 1.0 - rpm / engine_idle_rpm

	# Stall prevention: automatically add throttle when RPM is dangerously low.
	if rpm < engine_idle_rpm + rev_down_rate:
		throttle = maxf(throttle,
			minf(torque_balance_point * minf(engine_stall_prevention, 1.0)
				+ distance_below_idle * maxf(engine_stall_prevention - 1.0, 0.0), 1.0))
	# Rev limiter: cut throttle at the RPM ceiling.
	elif rpm > engine_max_rpm - rev_up_rate:
		throttle = 0.0

	var net_torque := throttle * rev_up_rate - rev_down_rate * (1.0 - throttle)
	rpm += net_torque * physics_steps_per_minute

	torque_data.torque_measure_rads = rpm_to_rads(net_torque)
	torque_data.rev_delta = rpm_to_rads(maxf(rev_up_rate, rev_down_rate))


static func rpm_to_rads(p_rpm: float) -> float:
	return p_rpm / 9.549297


static func rads_to_rpm(rads: float) -> float:
	return rads * 9.549297


# Smoothly ramps an analog value toward a target using separate attack/release rates.
static func ramp_analog(current: float, desired: float, on_rate: float, off_rate: float, ramp_scale: float) -> float:
	if current < desired - on_rate * ramp_scale:
		return current + on_rate * ramp_scale
	if current > desired + off_rate * ramp_scale:
		return current - off_rate * ramp_scale
	return desired


func handle_controls(delta_scale: float) -> void:
	var desired_steering := Input.get_axis("right", "left")
	var desired_accelerate := minf(Input.get_action_strength("up"), 1.0)
	var desired_decelerate := minf(Input.get_action_strength("down"), 1.0)
	var desired_handbrake := minf(Input.get_action_strength("jump"), 1.0)

	var desired_clutching := Input.get_action_strength("jump") \
		if controls.AST_shift_assistance > 0 \
		else Input.get_action_strength("sprint")

	# Auto-clutch: compute how far the engine RPM is below the target engagement point.
	var clutch_engagement_rpm := engine_idle_rpm + controls.AST_clutch_in_rpm_offset
	var distance_below_engagement_rpm := (clutch_engagement_rpm - rpm) / (clutch_engagement_rpm - engine_idle_rpm)

	if controls.AST_shift_assistance > 0:
		desired_clutching += clampf(distance_below_engagement_rpm, 0.0, 1.0)

	desired_clutching = minf(desired_clutching, 1.0)

	# During an automatic gear shift, override throttle/clutch to match engine speed to the new gear.
	if auto_throttle_blip_timer > 0:
		if rpm < wheel_driven_rpm:
			desired_accelerate = 1.0
			auto_clutch_engage_timer = auto_throttle_blip_timer
		else:
			desired_accelerate = 0.0
			auto_throttle_blip_timer -= delta_scale

	if auto_clutch_engage_timer > 0:
		desired_clutching = 1.0
		auto_clutch_engage_timer -= delta_scale

	analog_accelerate = ramp_analog(analog_accelerate, desired_accelerate, controls.OnThrottleRate, controls.OffThrottleRate, delta_scale)
	analog_decelerate = ramp_analog(analog_decelerate, desired_decelerate, controls.OnBrakeRate, controls.OffBrakeRate, delta_scale)
	analog_handbrake = ramp_analog(analog_handbrake, desired_handbrake, controls.OnHandbrakeRate, controls.OffHandbrakeRate, delta_scale)
	analog_clutching = ramp_analog(analog_clutching, desired_clutching, controls.OnClutchRate, controls.OffClutchRate, delta_scale)

	# Steering smoothing: pick a correction speed depending on whether the wheel is
	# turning into the desired direction, oversteering past it, or returning to centre.
	var steering_error := steering_input - desired_steering
	var steering_correction_speed: float

	if desired_steering > controls.KeyboardSteerSpeed:
		if steering_input > desired_steering:
			steering_correction_speed = controls.KeyboardReturnSpeed
		elif absf(steering_error) > 1.0:
			steering_correction_speed = controls.KeyboardCompensateSpeed
		else:
			steering_correction_speed = controls.KeyboardSteerSpeed
	elif desired_steering < -controls.KeyboardSteerSpeed:
		if steering_input < desired_steering:
			steering_correction_speed = controls.KeyboardReturnSpeed
		elif absf(steering_error) > 1.0:
			steering_correction_speed = controls.KeyboardCompensateSpeed
		else:
			steering_correction_speed = controls.KeyboardSteerSpeed
	else:
		steering_correction_speed = controls.KeyboardReturnSpeed

	steering_correction_speed *= delta_scale
	if steering_correction_speed > 0.0:
		steering_input -= steering_error / (absf(steering_error) / steering_correction_speed + 1.0)
	steering_input = clampf(steering_input, -1.0, 1.0)

	local_velocity = linear_velocity * global_transform.basis.orthonormalized()
	local_angular_velocity = angular_velocity * global_transform.basis.orthonormalized()

	# Lateral speed (sideways sliding). Zero it out if the slide opposes the steer direction
	# so we don't reduce steering authority when the driver is already correcting oversteer.
	var lateral_speed := absf(local_velocity.x)
	if (local_velocity.x > 0 and steering_input < 0) or (local_velocity.x < 0 and steering_input > 0):
		lateral_speed = 0.0

	steering_assistance_factor = 90.0 / maxf(max_steer_estimate, 1.0)

	# Blend from 0 (no assist at low speed) to 1 (full assist above threshold speed).
	var assist_blend := clampf(
		(linear_velocity.length() - controls.SteerAssistThreshold) / maxf(controls.SteerAssistThreshold, 1.0),
		0.0, 1.0)
	var forward_speed := maxf(local_velocity.z / (lateral_speed + 1.0), 0.0)
	# Reduces maximum steering angle at higher speeds to prevent spinning out.
	var max_steer_fraction := 1.0 / (forward_speed * assist_blend * (controls.SteerAmountDecay / steering_assistance_factor) + 1.0)

	steering_output = clampf(
		steering_input * max_steer_fraction
		+ local_velocity.normalized().x * assist_blend * (controls.SteeringAssistance * steering_assistance_factor)
		- local_angular_velocity.y * (controls.SteeringAssistanceAngular * steering_assistance_factor * assist_blend),
		-1.0, 1.0)


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	physics_state = state
	
	debug.add_or_update_debug("RPM: ", String.num(rpm, 0))
	debug.add_or_update_debug("Gear: ", String.num(gear, 0))
	debug.add_or_update_debug("km/h: ", String.num(linear_velocity.length() * 3.6, 0))

	var delta := state.step
	var physics_steps_per_minute := delta * 60.0
	var drivetrain_weight := 0.0
	wheel_driven_rpm = 0.0

	handle_controls(physics_steps_per_minute)

	# Normalise current RPM to 0–1 between idle and redline, for torque/decline curve sampling.
	var rpm_progress := (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	var torque_ratio := engine_torque_curve.sample_baked(rpm_progress) if engine_torque_curve != null else 1.0
	var decline_ratio := engine_decline_curve.sample_baked(rpm_progress) if engine_decline_curve != null else 1.0

	var rev_up_rate := engine_rev_up_speed * torque_ratio
	var rev_down_rate := engine_rev_down_speed * decline_ratio
	# Multiply gear ratio by 2 to convert from wheel rad/s to the RPM scale used internally.
	var combined_gear_ratio := gearbox() * 2.0

	# Apply gravity manually so wheel impulses can interact with it correctly this frame.
	apply_central_impulse(physics_state.total_gravity * mass * delta)

	# How strongly the engine RPM is pulled toward the wheel speed through the drivetrain.
	# Zero when in neutral or the clutch is fully pressed.
	var clutch_lock_strength := 0.0 if gear == 0 else \
		(maxf(engine_rev_up_speed, engine_rev_down_speed) + engine_rev_alignment_rate) * (1.0 - analog_clutching)
	clutch_lock_strength *= physics_steps_per_minute

	var clutch_lock_strength_rads := rpm_to_rads(clutch_lock_strength)
	# Normalises drive torque per unit of drivetrain weight so heavier drivetrains don't receive excess torque.
	var drivetrain_normalised_scale := engine_rev_up_speed / (engine_drive_force * physics_steps_per_minute)
	var engine_speed_rads := rpm_to_rads(rpm)

	handle_engine(delta, rev_up_rate, rev_down_rate)

	cached_drivetrain_weight = maxf(cached_drivetrain_weight, 1.0)

	handle_steering()

	# Accumulate weighted average spin rates across all wheels.
	var driven_wheel_spin_sum := 0.0
	var all_wheel_spin_sum := 0.0
	var all_wheel_travelled_sum := 0.0
	var driven_wheel_influence := 0.0

	for wheel in wheels:
		driven_wheel_influence += wheel.drivetrain_influence
		driven_wheel_spin_sum += wheel.spin * wheel.drivetrain_influence
		all_wheel_spin_sum += wheel.spin
		all_wheel_travelled_sum += wheel.distance_travelled_all_wheel_drive

	var average_all_wheel_spin := all_wheel_spin_sum / wheel_count
	var average_driven_wheel_spin := driven_wheel_spin_sum / driven_wheel_influence
	var average_all_wheel_travelled := all_wheel_travelled_sum / wheel_count

	driven_wheel_radius = 0.0
	# Ratio of coast (rev-down) to power (rev-up) rate — scales coast-side differential locking torque.
	var coast_to_torque_ratio := rev_down_rate / rev_up_rate

	predicted_linear_velocity = Vector3.ZERO
	predicted_angular_velocity = Vector3.ZERO

	for wheel in wheels:
		drivetrain_weight += wheel.drivetrain_influence
		wheel_driven_rpm += wheel.spin * wheel.drivetrain_influence

		# Estimate how much torque this wheel should receive from the engine through the gearbox.
		var wheel_torque_estimate: float = torque_data.rev_delta * wheel.drivetrain_influence * combined_gear_ratio / cached_drivetrain_weight / drivetrain_normalised_scale
		wheel_torque_estimate = minf(wheel_torque_estimate, clutch_lock_strength_rads * drivetrain_normalised_scale)

		# How much the wheel spins beyond what the clutch can hold (0 = no slip, >0 = wheelspin).
		var wheel_overdrive: float = maxf(wheel_torque_estimate / (torque_data.rev_delta / (combined_gear_ratio / physics_steps_per_minute)) - 1.0, 0.0)

		# Maximum torque the clutch can deliver to this wheel, reduced when the wheel is overdriving.
		var clutch_torque_limit: float = clutch_lock_strength_rads / (1.0 / (wheel_overdrive / wheel.tire_spin_resistance_rate + 1.0))
		# Clamp engine torque to what the clutch can physically deliver.
		var clamped_engine_torque: float = clampf(torque_data.torque_measure_rads, -clutch_torque_limit, clutch_torque_limit)

		# Choose whether all driven wheels share a common speed or each tracks its own.
		var reference_wheel_spin: float = average_driven_wheel_spin \
			if engine_drive_force_behavior == EngineDriveForceBehavior.WHEELS_UNSYNCED \
			else wheel.spin

		var predicted_wheel_spin: float
		var engine_wheel_speed_error: float

		# How far the engine speed is from matching the wheel speed through the gearbox.
		if gear < 0:
			predicted_wheel_spin = reference_wheel_spin + clamped_engine_torque / combined_gear_ratio - wheel.aligning_torque
			engine_wheel_speed_error = engine_speed_rads / combined_gear_ratio + predicted_wheel_spin
		else:
			predicted_wheel_spin = reference_wheel_spin - clamped_engine_torque / combined_gear_ratio - wheel.aligning_torque
			engine_wheel_speed_error = engine_speed_rads / combined_gear_ratio - predicted_wheel_spin

		# Final torque applied to the wheel: engine torque corrected toward perfect engine-wheel speed alignment.
		var aligning_torque_value: float = clampf(
			engine_wheel_speed_error * combined_gear_ratio - 1.0 + clamped_engine_torque,
			- clutch_torque_limit, clutch_torque_limit)
		var wheel_output_torque: float = aligning_torque_value * wheel.drivetrain_influence * combined_gear_ratio / cached_drivetrain_weight / drivetrain_normalised_scale

		wheel.drivetrain_overdrive = wheel_overdrive
		wheel.drivetrain_torque = - wheel_output_torque if gear < 0 else wheel_output_torque

		if wheel.differential_lock_target != null:
			# ── Axle differential ────────────────────────────────────────────────────────
			# A differential allows the two wheels on the same axle to spin at different
			# speeds (e.g. when cornering). Locking torque limits how different they can become.
			var axle_average_spin: float = (wheel.spin + wheel.differential_lock_target.spin) / 2.0
			var axle_wheel_travel_delta: float = wheel.distance_travelled - wheel.differential_lock_target.distance_travelled
			var axle_dampening_torque: float = wheel.spin - axle_average_spin
			var axle_spin_difference: float = wheel.spin - wheel.differential_lock_target.spin
			# Approaches 1 as the axle spins faster; reduces correction authority at speed.
			var axle_stabilisation: float = 1.0 - (1.0 / (absf(axle_average_spin * delta * wheel.wheel_radius) + 1.0))

			var axle_locking_torque: float = wheel.differential_locking_preload / 60.0
			if wheel_output_torque > 0.0:
				axle_locking_torque += wheel_output_torque * wheel.differential_locking_power * wheel.drivetrain_influence
			else:
				axle_locking_torque -= wheel_output_torque * wheel.differential_locking_coast * coast_to_torque_ratio * wheel.drivetrain_influence

			axle_wheel_travel_delta = clampf(axle_wheel_travel_delta, -axle_locking_torque, axle_locking_torque)
			axle_dampening_torque = clampf(axle_dampening_torque, -axle_locking_torque, axle_locking_torque)

			wheel.distance_travelled -= axle_wheel_travel_delta * axle_stabilisation
			wheel.drivetrain_torque -= axle_wheel_travel_delta + axle_dampening_torque \
				+ axle_spin_difference * wheel.differential_viscosity * delta * (wheel_overdrive + 1.0)

			# ── Central / AWD differential ────────────────────────────────────────────────
			# In an AWD system a central diff splits torque between front and rear axles.
			# This block limits how different the front/rear average speeds can become.

			var awd_dampening_torque: float = axle_average_spin - average_all_wheel_spin
			var awd_pair_average_travelled: float = (wheel.distance_travelled_all_wheel_drive + wheel.differential_lock_target.distance_travelled_all_wheel_drive) / 2.0
			var awd_travel_delta: float = awd_pair_average_travelled - average_all_wheel_travelled
			var awd_stabilisation: float = 1.0 - (1.0 / (absf(average_all_wheel_spin * delta * wheel.wheel_radius) + 1.0))
			var awd_locking_torque: float = diff_central_locking_preload / 60.0

			if wheel_output_torque > 0.0:
				awd_locking_torque += wheel_output_torque * diff_central_locking_power
			else:
				awd_locking_torque -= wheel_output_torque * diff_central_locking_coast * coast_to_torque_ratio

			awd_travel_delta = clampf(awd_travel_delta, -awd_locking_torque, awd_locking_torque)
			awd_dampening_torque = clampf(awd_dampening_torque, -awd_locking_torque, awd_locking_torque)

			wheel.distance_travelled_all_wheel_drive -= awd_travel_delta * awd_stabilisation
			wheel.drivetrain_torque -= awd_travel_delta + awd_dampening_torque

		# Braking torque = brake capacity x blended pedal input (foot brake + handbrake), scaled by ABS.
		wheel.drivetrain_braking = wheel.drivetrain_max_brake_torque \
			* minf(wheel.drivetrain_brake_bias * analog_decelerate + wheel.drivetrain_handbrake_bias * analog_handbrake, 1.0) \
			* get_abs_threshold()

		apply_impulse(wheel.impulse.force, wheel.impulse.position)
		predict_impulse(wheel.predicted_impulse.position, wheel.predicted_impulse.force / wheel_count)

		driven_wheel_radius += wheel.wheel_radius * wheel.drivetrain_influence

	driven_wheel_radius /= drivetrain_weight / 2.0

	cached_drivetrain_weight = drivetrain_weight

	# Convert accumulated wheel spin to RPM space and scale by gear ratio to get drivetrain RPM.
	wheel_driven_rpm *= rads_to_rpm(1.0) / drivetrain_weight * combined_gear_ratio
	if gear < 0:
		wheel_driven_rpm = - wheel_driven_rpm

	# Pull engine RPM toward wheel-driven RPM, limited by how tightly the clutch is engaged.
	var rpm_error := rpm - wheel_driven_rpm
	rpm -= clampf(rpm_error, -clutch_lock_strength, clutch_lock_strength)

	previous_rpm = rpm
	handle_audio()


# Returns a 0–1 brake multiplier; ABS reduces it when a wheel is locking up relative to the fastest wheel.
func get_abs_threshold() -> float:
	var fastest_wheel_spin := 0.0
	var abs_multiplier := 1.0

	for wheel in wheels:
		fastest_wheel_spin = maxf(fastest_wheel_spin, absf(wheel.spin))

	for wheel in wheels:
		if wheel.abs_enabled:
			abs_multiplier -= absf(absf(wheel.spin + wheel.aligning_torque) - fastest_wheel_spin) \
				* wheel.abs_sensitivity * wheel_count / 100.0

	# Handbrake bypasses ABS: lerp toward 1 (full braking) as the handbrake is pulled.
	return lerpf(maxf(abs_multiplier, 0.0), 1.0, analog_handbrake)


# Rotates each steered wheel to the correct Ackermann angle for the current steering output.
func handle_steering() -> void:
	if absf(steering_output) == 0.0:
		return

	# Turn radius of the vehicle centre: larger steering_output = tighter turn.
	var turn_radius := steering_radius / absf(steering_output)
	if steering_output < 0.0:
		turn_radius *= -1.0

	for wheel in steering_wheels:
		# Ackermann formula: each wheel steers to a slightly different angle so all four
		# wheel arcs share the same turning centre point.
		wheel.rotation_degrees.y = rad_to_deg(
			atan((-steering_pivot_z + wheel.position.z) / (turn_radius - wheel.position.x)))


func handle_audio() -> void:
	if not engine_on or not engine_off:
		return

	var pitch_scale := maxf(rpm, 800.0) / 6000.0

	engine_on.volume_db = maxf(linear_to_db(throttle), -80.0)
	engine_off.volume_db = maxf(linear_to_db(1.0 - throttle), -80.0)
	engine_on.max_db = engine_on.volume_db
	engine_off.max_db = engine_off.volume_db

	engine_on.pitch_scale = pitch_scale
	engine_off.pitch_scale = pitch_scale


class TorqueData:
	# Engine torque expressed in radians/s — used to drive wheel spin each physics step.
	var torque_measure_rads: float = 0.0
	# Maximum rev change per step (the larger of rev-up and rev-down rates), in rad/s.
	var rev_delta: float = 0.0
