package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Main.Subsystems.AprilSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivebase;



@TeleOp(name="Drivebase Only TeleOp", group="Linear OpMode")
public class DrivebaseOnly extends LinearOpMode {

    // Setup Gamepads
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    //init
    @Override
    public void runOpMode() {

        // Define gamepads
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Create the subsystems
        Drivebase Drivebase = new Drivebase(hardwareMap);
        AprilSystem AprilSystem = new AprilSystem(hardwareMap, telemetry);

        // Configure telemetry
        telemetry.setMsTransmissionInterval(100);

        // Create telemetry manager
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Go time
        waitForStart();

        while (opModeIsActive()) {
            // Send gamepad inputs to the subsystems
            Drivebase.ProcessInput(gamepadEx1, AprilSystem);

            // Create telemetry
            if (Drivebase.speedBool) {
                panelsTelemetry.addLine("SLOW MODE ENABLED");
            }
            // Add panels telemetry
            panelsTelemetry.addData("Pinpoint Heading", Drivebase.heading1);
            panelsTelemetry.addData("REV IMU Heading", Drivebase.heading2);

            // Update telemetry
            panelsTelemetry.update(telemetry);
        }
    }
}
