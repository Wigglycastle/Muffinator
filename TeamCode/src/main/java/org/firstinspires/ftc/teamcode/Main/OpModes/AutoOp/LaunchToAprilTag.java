package org.firstinspires.ftc.teamcode.Main.OpModes.AutoOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Main.Subsystems.AprilSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivebase;
@Autonomous(name="AUTO-GOTO April Tag and Launch", group="Linear OpMode")
public class LaunchToAprilTag extends LinearOpMode {
    //init
    @Override
    public void runOpMode() {
        // Create telemetry
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create the subsystems
        Drivebase Drivebase = new Drivebase(hardwareMap);
        ArtifactSystem ArtifactSystem = new ArtifactSystem(hardwareMap);
        //LightingSystem LightingSystem = new LightingSystem(hardwareMap);
        AprilSystem AprilSystem = new AprilSystem(hardwareMap, telemetry);

        // Configure telemetry
        telemetry.setMsTransmissionInterval(100);

        // Start Lights
        //LightingSystem.PreGameLights();

        // Go time
        waitForStart();

        // Change Lights
        //LightingSystem.MidGameLights();

        while (opModeIsActive()) {
            if (Drivebase.NavigateToAprilTag(AprilSystem,22)) {
                break;
            }
        }
        ArtifactSystem.FlywheelPowerTo(1);
        ArtifactSystem.StaggeredFeed();
        ArtifactSystem.FlywheelPowerTo(0);
        ArtifactSystem.StopFeed();
    }
}