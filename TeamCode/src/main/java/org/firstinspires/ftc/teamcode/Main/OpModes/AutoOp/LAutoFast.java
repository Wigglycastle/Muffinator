
package org.firstinspires.ftc.teamcode.Main.OpModes.AutoOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.Subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.Main.Subsystems.LightingSystem;
import org.firstinspires.ftc.teamcode.Main.Utils.AutoStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "12.3 BLUE Auto", group = "Autonomous")
@Configurable // Panels
public class LAutoFast extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private LightingSystem lightingSystem;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final double INTAKE_SPEED = 0.5;
    private ArtifactSystem artifactSystem;
    private final double outtakeTime = 1.75;
    @Override
    public void init() {
        lightingSystem = new LightingSystem(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        artifactSystem = new ArtifactSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120.43636363636362, 127.48531468531466, Math.toRadians(36)));

        paths = new Paths(follower); // Build paths
        lightingSystem.SetLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void start() {
        lightingSystem.SetLights(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        artifactSystem.Update();
        Drawing.drawDebug(follower);
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(24.369, 128.291),
                                    new Pose(37.175, 106.129),
                                    new Pose(51.155, 106.741)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(51.155, 106.741),
                                    new Pose(44.647, 81.428),
                                    new Pose(18.529, 83.782)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.529, 83.782),
                                    new Pose(43.269, 108.419),
                                    new Pose(51.759, 106.943)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(51.759, 106.943),
                                    new Pose(56.176, 56.738),
                                    new Pose(19.133, 60.017)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.133, 60.017),
                                    new Pose(46.835, 86.508),
                                    new Pose(51.155, 106.943)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(51.155, 106.943),
                                    new Pose(59.412, 35.124),
                                    new Pose(43.094, 34.893),
                                    new Pose(13.952, 35.733)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.952, 35.733),
                                    new Pose(36.179, 54.928),
                                    new Pose(51.193, 106.985)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(51.193, 106.985),
                                    new Pose(14.582, 60.700),
                                    new Pose(7.049, 13.897)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(7.049, 13.897),
                                    new Pose(33.566, 81.136),
                                    new Pose(51.280, 106.951)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(144))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                artifactSystem.setFlywheel(true);
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
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 101;
                }
                break;

            case 101:
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    follower.followPath(paths.Path4, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 102;
                }
                break;

            case 102:
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    follower.followPath(paths.Path6, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 103;
                }
                break;

            case 103:
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.INTAKE);
                    follower.followPath(paths.Path8, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.OUTTAKE);
                    stateTimer.reset();
                    pathState = 104;
                }
                break;

            case 104:
                if (stateTimer.seconds() > outtakeTime) {
                    artifactSystem.setState(ArtifactSystem.ArtifactSystemStates.IDLE);
                    artifactSystem.setFlywheel(false);
                    pathState = 10;
                }
                break;

            case 10:
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
    