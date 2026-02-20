package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem.ArtifactSystemStates.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Main.Subsystems.AprilSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Main.Subsystems.LightingSystem;
import org.firstinspires.ftc.teamcode.Main.Utils.AutoStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.DrivePowers;


@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

    // Setup Gamepads
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    //init
    @Override
    public void runOpMode() {

        // Define gamepads
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Create telemetry
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create the subsystems
        Drivebase Drivebase = new Drivebase(hardwareMap);
        ArtifactSystem ArtifactSystem = new ArtifactSystem(hardwareMap);
        LightingSystem LightingSystem = new LightingSystem(hardwareMap);
        AprilSystem AprilSystem = new AprilSystem(hardwareMap, telemetry);

        // Configure telemetry
        telemetry.setMsTransmissionInterval(100);

        //Start Lights
        LightingSystem.SetLights(AutoStorage.color);

        // Go time
        waitForStart();

        while (opModeIsActive()) {
            // Read controllers
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Send gamepad inputs to the subsystems
            if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                ArtifactSystem.setState(HUMAN_INTAKE);
            } else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                ArtifactSystem.setState(INTAKE);
            } else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                ArtifactSystem.setState(OUTTAKE);
            } else if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                ArtifactSystem.setState(FLUSH);
            } else {
                ArtifactSystem.setState(IDLE);
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                ArtifactSystem.setFlywheel(true);
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                ArtifactSystem.setFlywheel(false);
            }

            ArtifactSystem.Update();

            // Calculate wheel outputs
            DrivePowers gamepadPowers = Drivebase.ProcessInput(gamepadEx1);
            DrivePowers aprilPowers = null;
            if (gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                aprilPowers = AprilSystem.CheckForTag(30, -1);
            }
            if (aprilPowers != null) {
                Drivebase.SetMotorPowers(aprilPowers);
                LightingSystem.SetLights(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            } else {
                Drivebase.SetMotorPowers(gamepadPowers);
                LightingSystem.SetLights(AutoStorage.color);
            }

            // Create telemetry
            if (Drivebase.speedBool) {
                telemetry.addLine("SLOW MODE ENABLED");
            }
            telemetry.addLine("Heading Pinpoint:" + Drivebase.heading1);
            telemetry.addLine("Heading IMU:" + Drivebase.heading2);
            // Add panels telemetry
            panelsTelemetry.addData("Pinpoint Heading", Drivebase.heading1);
            panelsTelemetry.addData("REV IMU Heading", Drivebase.heading2);
            panelsTelemetry.addData("FlywheelRPM", ArtifactSystem.flywheelRPM);

            // Update telemetry
            panelsTelemetry.update(telemetry);
        }
    }
}
