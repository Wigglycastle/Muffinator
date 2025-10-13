package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivebase;


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
        Arm Arm = new Arm(hardwareMap);

        // Configure telemetry
        telemetry.setMsTransmissionInterval(100);

        // Go time
        waitForStart();

        while (opModeIsActive()) {
            // Send gamepad inputs to the subsystems
            Drivetrain.ProcessInput(gamepadEx1);
            Arm.ProcessInput(gamepadEx1);

            // Create and send telemetry to robot
            telemetry.addLine("Claw Degrees:" + Arm.getClawRot());
            telemetry.addLine("Wrist Degrees:" + Arm.getWristRot());
            telemetry.addLine("Arm Ticks:" + Arm.getArmRot());
            telemetry.addLine("State:" + Arm.getState());
            if (Drivetrain.GetSlowMode()) {
                telemetry.addLine("SLOW MODE ENABLED");
            }
            telemetry.update();

        }
    }
}
