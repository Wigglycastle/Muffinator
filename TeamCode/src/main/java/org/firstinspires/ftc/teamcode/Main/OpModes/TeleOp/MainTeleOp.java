package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Climb;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Main.Subsystems.LightingSystem;


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

        // Create the subsystems
        Drivebase Drivetrain = new Drivebase(hardwareMap);
        ArtifactSystem ArtifactSystem = new ArtifactSystem(hardwareMap);
        Climb Climb = new Climb(hardwareMap);
        LightingSystem LightingSystem = new LightingSystem(hardwareMap);

        // Configure telemetry
        telemetry.setMsTransmissionInterval(100);

        // Start Lights
        LightingSystem.PreGameLights();

        // Go time
        waitForStart();

        // Change Lights
        LightingSystem.MidGameLights();

        while (opModeIsActive()) {
            // Send gamepad inputs to the subsystems
            Drivetrain.ProcessInput(gamepadEx1);
            ArtifactSystem.ProcessInput(gamepadEx2);
            Climb.ProcessInput(gamepadEx2);

            // Create and send telemetry to robot
            if (Drivetrain.speedBool) {
                telemetry.addLine("SLOW MODE ENABLED");
            }
            telemetry.update();

        }
    }
}
