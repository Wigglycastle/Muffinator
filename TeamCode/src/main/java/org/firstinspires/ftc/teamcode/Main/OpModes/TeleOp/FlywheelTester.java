package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name="Flywheel Test", group="Linear OpMode")
public class FlywheelTester extends LinearOpMode {

    // Setup Gamepads
    public GamepadEx gamepadEx1;
    private double targetRPM = 0;
    private final double MOTOR_TPR = 28;

    //init
    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        MotorEx flywheelMotor = new MotorEx(hardwareMap, "FlwM");
        flywheelMotor.setRunMode(Motor.RunMode.VelocityControl);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setVeloCoefficients(0.25, 0.002, 0.01);
        flywheelMotor.setFeedforwardCoefficients(0.1, 0.0005, 0.0);
        Motor.Encoder encoder = flywheelMotor.encoder;

        encoder.reset();

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
               targetRPM += 500;
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                targetRPM -= 500;
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                targetRPM = 0;
            }

            targetRPM = Math.max(-6000, Math.min(targetRPM, 6000));
            double targetTPS = targetRPM * MOTOR_TPR / 60.0;
            flywheelMotor.setVelocity(targetTPS);
            double motorRPM = encoder.getCorrectedVelocity() * 60.0 / MOTOR_TPR;
            panelsTelemetry.addData("Wheel RPM", motorRPM);
            panelsTelemetry.addData("Target RPM", targetRPM);
            panelsTelemetry.update(telemetry);
        }
    }
}
