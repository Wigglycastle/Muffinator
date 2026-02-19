
package org.firstinspires.ftc.teamcode.Main.OpModes.AutoOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Utils.AutoStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "9.3 RED Auto", group = "Autonomous")
@Configurable // Panels
public class RAuto extends OpMode {
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
        follower.setStartingPose(new Pose(86.6013986013986, 9.264335664335658, Math.toRadians(90)));

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
                                    new Pose(86.601, 9.264),
                                    new Pose(85.887, 88.474),
                                    new Pose(91.636, 106.137)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.636, 106.137),
                                    new Pose(85.952, 88.357),
                                    new Pose(83.983, 84.185)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.983, 84.185),

                                    new Pose(128.291, 83.379)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.291, 83.379),
                                    new Pose(86.117, 88.101),
                                    new Pose(91.435, 105.936)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.435, 105.936),
                                    new Pose(86.227, 87.234),
                                    new Pose(83.580, 59.413)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.580, 59.413),

                                    new Pose(128.895, 59.614)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.895, 59.614),
                                    new Pose(86.326, 87.698),
                                    new Pose(92.039, 105.734)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.039, 105.734),
                                    new Pose(85.421, 87.773),
                                    new Pose(84.442, 35.531)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.442, 35.531),

                                    new Pose(129.483, 35.250)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.483, 35.250),
                                    new Pose(85.589, 86.858),
                                    new Pose(91.674, 105.978)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

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
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE); // Intake during path 6
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
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
        AutoStorage.heading = follower.getHeading() + Math.toRadians(90);
        AutoStorage.color = RevBlinkinLedDriver.BlinkinPattern.RED;
    }
}
    