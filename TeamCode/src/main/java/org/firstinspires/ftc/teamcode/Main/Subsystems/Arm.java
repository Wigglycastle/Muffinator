package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


public class Arm {

    private final MotorEx swingArmMotor;
    private final SimpleServo clawServo;
    private final SimpleServo wristServo;
    private double servo1Rot;
    private double servo2Rot;
    private int state = 0;
    private int motorRot = -100;

    public Arm(HardwareMap hardwareMap) {

        // Define motors
        swingArmMotor = new MotorEx(hardwareMap, "swingArm", Motor.GoBILDA.RPM_84);
        clawServo = new SimpleServo(hardwareMap, "claw", -90, 90);
        wristServo = new SimpleServo(hardwareMap, "wrist", -180, 180);

        // Set Motor config
        swingArmMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        swingArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        swingArmMotor.setRunMode(Motor.RunMode.PositionControl);
        swingArmMotor.setPositionCoefficient(0.01);
        swingArmMotor.setPositionTolerance(5);
    }

    public void ProcessInput(GamepadEx gamepadEx) {

        // Get inputs from gamepad
        gamepadEx.readButtons();

        // Process gamepad inputs
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            servo1Rot = servo1Rot + 10;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            servo1Rot = servo1Rot - 10;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            servo2Rot = servo2Rot + 10;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            servo2Rot = servo2Rot - 10;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            motorRot = motorRot + 200;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            motorRot = motorRot - 200;
        }

        // State ticks up by 1 every time B is pressed until it reaches 5
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            state = state + 1;
            if (state >= 5) {
                state = 0;
            }
        }

        if (state == 1) {
            motorRot = -100;
        }
        if (state == 2) {
            motorRot = -2000;
        }
        if (state == 3) {
            motorRot = -4000;
        }
        if (state == 4) {
            motorRot = -6200;
            servo1Rot = -30;
            servo2Rot = -80;
        }

        // Send power to motors and servos
        clawServo.turnToAngle(servo1Rot);
        wristServo.turnToAngle(servo2Rot);
        swingArmMotor.setTargetPosition(motorRot);
        swingArmMotor.set(1);
    }

    public double getClawRot() {
        return servo1Rot;
    }

    public double getWristRot() {
        return servo2Rot;
    }

    public double getArmRot() {
        return motorRot;
    }

    public double getState() {
        return state;
    }

}
