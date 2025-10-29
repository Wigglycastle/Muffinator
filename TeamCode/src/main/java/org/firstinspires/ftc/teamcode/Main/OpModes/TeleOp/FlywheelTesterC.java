package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name="Flywheel Test - Calibrated", group="Linear OpMode")
public class FlywheelTesterC extends LinearOpMode {

    public GamepadEx gamepadEx1;
    private double targetRPM = 0;
    // you've measured encoder counts per output-shaft revolution as 28
    private final double MOTOR_TPR = 28.0;

    @Override
    public void runOpMode() {

        gamepadEx1 = new GamepadEx(gamepad1);
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        MotorEx flywheelMotor = new MotorEx(hardwareMap, "FlwM");
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // --- 1) quick open-loop measurement to compute kV ---
        flywheelMotor.setRunMode(Motor.RunMode.RawPower);
        sleep(100); // let controller settle

        // safely ramp to full power and measure steady velocity
        flywheelMotor.set(1.0);
        sleep(800); // let it spin up and settle
        double freeTPS = flywheelMotor.getVelocity(); // ticks/sec measured at full power
        flywheelMotor.set(0.0);
        sleep(200);

        // compute kV as 1 / (ticks/sec at full power)
        // protects against divide-by-zero
        double kV = (freeTPS > 50.0) ? 1.0 / freeTPS : 0.00036; // fallback estimate

        // small static term to overcome stiction; tune between 0.03 - 0.15 if needed
        double kS = 0.08;

        // set feedforward (kS, kV, kA)
        flywheelMotor.setFeedforwardCoefficients(kS, kV, 0.0);

        // conservative PID to start with (feedforward will do the heavy lifting)
        double kP = 0.002;
        double kI = 0.0;
        double kD = 0.0005;
        flywheelMotor.setVeloCoefficients(kP, kI, kD);

        // switch into closed-loop velocity control
        flywheelMotor.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                targetRPM += 100; // smaller step gives more control
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                targetRPM -= 100;
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                targetRPM = 0;
            }

            targetRPM = Math.max(-6000, Math.min(targetRPM, 6000));
            double targetTPS = targetRPM * MOTOR_TPR / 60.0;

            // command ticks/sec
            flywheelMotor.setVelocity(targetTPS);

            // measured ticks/sec and convert to RPM for telemetry
            double measuredTPS = flywheelMotor.getVelocity();
            double motorRPM = measuredTPS * 60.0 / MOTOR_TPR;

            panelsTelemetry.addData("Target RPM", targetRPM);
            panelsTelemetry.addData("Measured RPM", String.format("%.1f", motorRPM));
            panelsTelemetry.addData("Target TPS", String.format("%.1f", targetTPS));
            panelsTelemetry.addData("Measured TPS", String.format("%.1f", measuredTPS));
            panelsTelemetry.addData("freeTPS (cal)", String.format("%.1f", freeTPS));
            panelsTelemetry.addData("kS (set)", kS);
            panelsTelemetry.addData("kV (cal)", String.format("%.6f", kV));
            panelsTelemetry.update(telemetry);
        }
    }
}
