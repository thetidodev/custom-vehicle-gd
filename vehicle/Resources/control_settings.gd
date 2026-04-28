class_name ControlSettingsGD
extends Resource

enum ASTShiftAssistanceEnum {
	None = 0,
	AutoClutch = 1,
}

@export var AST_shift_assistance: ASTShiftAssistanceEnum = ASTShiftAssistanceEnum.None
@export var AST_clutch_in_rpm_offset: float = 2000.0
@export var AST_upshift_threshold: float = -250.0
@export var AST_downshift_threshold: float = -500.0
@export var AST_standstill_threshold: float = -100.0
@export var AST_shifting_clutch_out_time: int = 20
@export var AST_shifting_off_throttle_time: int = 21
@export var AST_reverse_delay: int = 30
@export var AST_instant_reverse: bool = false

@export var SteerSensitivity: float = 1.0
@export var SteerAssistThreshold: float = 5.0
@export var KeyboardSteerSpeed: float = 0.05
@export var KeyboardReturnSpeed: float = 0.05
@export var KeyboardCompensateSpeed: float = 0.1

@export var SteerAmountDecay: float = 0.4
@export var SteeringAssistance: float = 0.0
@export var SteeringAssistanceAngular: float = 0.0

@export var OnThrottleRate: float = 0.2
@export var OffThrottleRate: float = 0.2

@export var OnBrakeRate: float = 0.05
@export var OffBrakeRate: float = 0.1

@export var OnHandbrakeRate: float = 0.2
@export var OffHandbrakeRate: float = 0.2

@export var OnClutchRate: float = 0.2
@export var OffClutchRate: float = 0.2
