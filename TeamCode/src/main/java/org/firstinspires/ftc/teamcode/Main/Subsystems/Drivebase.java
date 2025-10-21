package org.firstinspires.ftc.teamcode.Main.Subsystems;

// The basics

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class Drivebase {

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    public boolean speedBool;


    private final GoBildaPinpointDriver pinpoint;
    private final IMU imu;
    private double speedFactor = 1;
    private double crabSpeedFactor;
    public double heading1;
    public double heading2;

    public Drivebase(HardwareMap hardwareMap) {

        // For heading
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        //TODO: Get heading data from end of auto, set to temp value
        pinpoint.setHeading(90, AngleUnit.DEGREES);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Define motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Set Motor Directions
        // Set Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set Motor Behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set crab crawl speed (Add adjustment later on?)
        crabSpeedFactor = 0.4;
    }
    public void ProcessInput(GamepadEx gamepad, AprilSystem aprilSystem) {
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            aprilSystem.CheckForTag(90, -1);
        }
        else if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            aprilSystem.CheckForTag(60, -1);
        }
        else {
            aprilSystem.backLeftPower = 0;
            aprilSystem.frontLeftPower = 0;
            aprilSystem.backRightPower = 0;
            aprilSystem.frontRightPower = 0;
        }

        // Get heading data from pinpoint
        pinpoint.update();
        heading1 = pinpoint.getHeading(AngleUnit.RADIANS);
        heading2 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Set speed factor, when A is pressed the robot goes into a fine adjustment mode, on a toggle
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            speedBool = !speedBool;
            speedFactor = speedBool ? 0.5 : 1;
        }

        // Reset Position
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
        }

        // This is for crab crawl
        double vertical = gamepad.getButton(GamepadKeys.Button.DPAD_UP) ? crabSpeedFactor : (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) ? -crabSpeedFactor : 0);
        double horizontal = gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? crabSpeedFactor : (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) ? -crabSpeedFactor : 0);

        // Rotate input vector by the negative heading and apply speed factor
        double x    =  -gamepad.getLeftY() * speedFactor;
        double y    =  gamepad.getLeftX() * speedFactor;
        double rx   =  gamepad.getRightX() * speedFactor;

        // Rotate input vector by the negative heading
        double rotX = x * Math.cos(-heading1) - y * Math.sin(-heading1) - horizontal;
        double rotY = x * Math.sin(-heading1) + y * Math.cos(-heading1) + vertical;

        // Calculate, normalize, multiply by speed factor, and send power to motors
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontDrive.setPower(( rotY + rotX + rx) / denom + aprilSystem.frontLeftPower);
        leftBackDrive.setPower(( rotY - rotX + rx) / denom + aprilSystem.backLeftPower);
        rightFrontDrive.setPower(( rotY - rotX - rx) / denom + aprilSystem.frontRightPower);
        rightBackDrive.setPower(( rotY + rotX - rx) / denom + aprilSystem.backRightPower);
        }
    }
