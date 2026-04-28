extends Node3D

@export var wheels: Array[Wheel]
@export var rolling: AudioStreamPlayer3D
@export var rub: AudioStreamPlayer3D
@export var skid: AudioStreamPlayer3D

func _physics_process(_delta: float) -> void:
	var total_rolling := 0.0
	var total_skidding := 0.0
	var total_rubbing := 0.0

	for wheel in wheels:
		total_rolling += absf(wheel.spin)
		total_skidding += absf(wheel.output_skidding)
		total_rubbing += absf(wheel.output_stressing)

	total_skidding = maxf(total_skidding - 20.0, 0.0) * 0.25
	total_rubbing = maxf(total_rubbing - 10.0, 0.0)

	rolling.pitch_scale = 1.0 + total_rolling / 1000.0
	rolling.volume_db = clampf(linear_to_db(maxf(total_rolling / 1000.0, 0.0) * 0.75), -80.0, 0.0)
	rolling.max_db = rolling.volume_db

	rub.volume_db = clampf(linear_to_db(maxf(total_rubbing - total_skidding, 0.0) * 0.75 / 300.0), -80.0, 0.0)
	rub.max_db = rub.volume_db

	skid.volume_db = clampf(linear_to_db(maxf(total_skidding, 0.0) / 30.0 * 0.75), -80.0, 0.0)
	skid.max_db = skid.volume_db
