extends Node3D

@export var car: RigidBody3D
@export var target: Node3D
@export var rotation_point: Node3D
@export var spring_arm: SpringArm3D
@export var camera: Camera3D
@export var default_fov: float = 75.0
@export var default_spring_length: float = 5.0

var cameras: Array[Camera3D] = []
var active_cam: int = 5

func _ready() -> void:
	for node in get_tree().get_nodes_in_group("Camera"):
		cameras.append(node as Camera3D)
	switch_camera()

func switch_camera() -> void:
	if active_cam == cameras.size() - 1:
		active_cam = 0
	else:
		active_cam += 1

	for cam in cameras:
		cam.current = false
	cameras[active_cam].current = true
	print(active_cam)

func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		spring_arm.rotate_x(event.relative.y * 0.001)
		rotation_point.rotate_y(-event.relative.x * 0.001)
		spring_arm.rotation = Vector3(clamp(spring_arm.rotation.x, deg_to_rad(-80), deg_to_rad(80)), PI, 0)

func _physics_process(_delta: float) -> void:
	if Input.is_action_just_pressed("esc"):
		if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		else:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

	if Input.is_action_just_pressed("cam"):
		switch_camera()

	spring_arm.rotate_x(Input.get_axis("rUp", "rDown") * 0.04)
	rotation_point.rotate_y(Input.get_axis("rRight", "rLeft") * 0.04)
	spring_arm.rotation = Vector3(clamp(spring_arm.rotation.x, deg_to_rad(-80), deg_to_rad(80)), PI, 0)

	global_position = global_position.lerp(target.global_position, 0.2)
	global_rotation.y = target.global_rotation.y

	var speed_factor: float = clamp((260.0 + car.linear_velocity.length() * 3.6) / 300.0, 1.0, 1.5)
	camera.fov = default_fov * speed_factor
	spring_arm.spring_length = default_spring_length * 1.0 / speed_factor
