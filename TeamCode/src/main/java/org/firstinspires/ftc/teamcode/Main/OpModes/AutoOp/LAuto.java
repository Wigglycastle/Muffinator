
package org.firstinspires.ftc.teamcode.Main.OpModes.AutoOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Utils.AutoStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "9.3 BLUE Auto", group = "Autonomous")
@Configurable // Panels
public class LAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final double INTAKE_SPEED = 0.5;
    private ArtifactSystem artifactSystem;
    int pulseCounter = 0;
    private final float outtakeTime = 2;
    @Override
    public void init() {
        //LightingSystem LightingSystem = new LightingSystem(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        artifactSystem = new ArtifactSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.5, 9.264335664335658, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        //LightingSystem.SetLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        artifactSystem.Update();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.500, 9.264),
                                    new Pose(57.691, 70.952),
                                    new Pose(53.975, 100.498)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(143))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.975, 100.498),
                                    new Pose(66.416, 87.350),
                                    new Pose(59.815, 86.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.815, 86.400),

                                    new Pose(15.105, 85.997)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.105, 85.997),
                                    new Pose(53.289, 82.664),
                                    new Pose(53.975, 100.699)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.975, 100.699),
                                    new Pose(63.469, 91.262),
                                    new Pose(59.211, 61.024)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.211, 61.024),

                                    new Pose(13.091, 61.024)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.091, 61.024),
                                    new Pose(54.102, 70.579),
                                    new Pose(53.773, 100.297)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.773, 100.297),
                                    new Pose(63.871, 83.946),
                                    new Pose(59.267, 37.143)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.267, 37.143),

                                    new Pose(13.276, 37.344)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.276, 37.344),
                                    new Pose(48.330, 63.898),
                                    new Pose(54.013, 100.540)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                artifactSystem.setFlywheel(true); // Rev flywheel during path 1
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 100;
                }
                break;

            case 100:
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_SPEED);
                    follower.followPath(paths.Path3, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path4, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // End of path 4 - outtake
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 101;
                }
                break;

            case 101: // Outtake after path 4
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_SPEED);
                    follower.followPath(paths.Path6, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    //artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE); TODO: Disabled for consistency
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    // End of path 7 - outtake
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 102;
                }
                break;

            case 102: // Outtake after path 7
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path8, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_SPEED);
                    follower.followPath(paths.Path9, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE); // Intake during path 9
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path10, true);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 103;
                }
                break;

            case 103: // Final outtake
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    artifactSystem.setFlywheel(false); // Stop flywheel when done
                    pathState = 11; // Done
                }
                break;

            case 11:
                // Autonomous complete
                break;
        }

        return pathState;
    }

    @Override
    public void stop() {
        AutoStorage.heading = follower.getHeading() + Math.toRadians(-90);
        AutoStorage.color = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    }
}
    