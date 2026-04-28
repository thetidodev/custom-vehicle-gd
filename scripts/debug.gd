class_name Debug
extends Control

@export var list: VBoxContainer

var debug_labels: Dictionary = {}

func _physics_process(_delta: float) -> void:
	add_or_update_debug("FPS", str(Engine.get_frames_per_second()))

func add_or_update_debug(label_name: String, value: String) -> void:
	if debug_labels.has(label_name):
		debug_labels[label_name].text = label_name + " : " + value
	else:
		var label = Label.new()
		label.text = label_name + " : " + value
		debug_labels[label_name] = label
		list.add_child(label)
