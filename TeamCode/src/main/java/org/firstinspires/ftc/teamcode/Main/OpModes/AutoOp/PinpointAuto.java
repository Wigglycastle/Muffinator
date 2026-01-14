
package org.firstinspires.ftc.teamcode.Main.OpModes.AutoOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PinpointAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ArtifactSystem artifactSystem;
    private final ElapsedTime stateTimer = new ElapsedTime();
    int pulseCounter = 0;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;

        artifactSystem = new ArtifactSystem(hardwareMap);
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
                                    new Pose(56.996, 9.264),
                                    new Pose(53.687, 68.850),
                                    new Pose(41.085, 111.172)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(41.085, 111.172),
                                    new Pose(48.422, 100.477),
                                    new Pose(47.731, 83.580)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.731, 83.580),

                                    new Pose(18.327, 83.983)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.327, 83.983),
                                    new Pose(32.957, 94.550),
                                    new Pose(41.085, 111.172)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(41.085, 111.172),
                                    new Pose(41.717, 81.594),
                                    new Pose(47.731, 59.211)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.731, 59.211),

                                    new Pose(18.529, 59.614)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.529, 59.614),
                                    new Pose(40.495, 58.520),
                                    new Pose(40.884, 111.575)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(40.884, 111.575),
                                    new Pose(54.204, 63.605),
                                    new Pose(47.385, 34.927)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.385, 34.927),

                                    new Pose(18.915, 35.855)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.915, 35.855),
                                    new Pose(44.606, 69.124),
                                    new Pose(40.720, 111.415)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

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
                    // End of path 1 - start outtake
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 100; // Outtake sub-state
                }
                break;

            case 100: // Outtake after path 1
                if (stateTimer.seconds() > 1.0) { // Outtake duration
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE); // Intake during path 3
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE); // Stop intake
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
                if (stateTimer.seconds() > 1.0) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE); // Intake during path 6
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
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
                if (stateTimer.seconds() > 1.0) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path8, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE); // Intake during path 9
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    follower.followPath(paths.Path10, true);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // End of path 10 - final outtake
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 103;
                }
                break;

            case 103: // Final outtake
                if (stateTimer.seconds() > 1.0) {
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
}
    