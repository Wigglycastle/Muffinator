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
import org.firstinspires.ftc.teamcode.Main.Utils.DrivePowers;
import org.firstinspires.ftc.teamcode.Main.Utils.HeadingStorage;


public class Drivebase {

    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;

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
        pinpoint.setHeading(HeadingStorage.heading, AngleUnit.RADIANS);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Define motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Set Motor Directions
        // Set Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set Motor Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set crab crawl speed (Add adjustment later on?)
        crabSpeedFactor = 0.4;
    }
    public DrivePowers ProcessInput(GamepadEx gamepad) {
        // Get heading data from pinpoint
        pinpoint.update();
        heading1 = pinpoint.getHeading(AngleUnit.RADIANS);
        heading2 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Set speed factor, when Back is pressed the robot goes into a fine adjustment mode, on a toggle
        if (gamepad.wasJustPressed(GamepadKeys.Button.BACK)) {
            speedBool = !speedBool;
            speedFactor = speedBool ? 0.5 : 1;
        }

        // Reset Position
        if (gamepad.wasJustPressed(GamepadKeys.Button.START)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
        }

        // This is for crab crawl
        double vertical = gamepad.getButton(GamepadKeys.Button.DPAD_UP) ? crabSpeedFactor :
                (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) ? -crabSpeedFactor : 0);
        double horizontal = gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? crabSpeedFactor :
                (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) ? -crabSpeedFactor : 0);

        // Rotate input vector by the negative heading and apply speed factor
        double x    =  -gamepad.getLeftY() * speedFactor;
        double y    =  gamepad.getLeftX() * speedFactor;
        double rx   =  gamepad.getRightX() * speedFactor;

        // Rotate input vector by the negative heading
        double rotX = x * Math.cos(-heading1) - y * Math.sin(-heading1) - horizontal;
        double rotY = x * Math.sin(-heading1) + y * Math.cos(-heading1) + vertical;

        // Calculate, normalize, multiply by speed factor, and send power to motors
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denom;
        double backLeftPower = (rotY - rotX + rx) / denom;
        double frontRightPower = (rotY - rotX - rx) / denom;
        double backRightPower = (rotY + rotX - rx) / denom;
        return new DrivePowers(frontLeftPower,backLeftPower,frontRightPower,backRightPower);
        }
        public void SetMotorPowers(DrivePowers drivePowers) {
            frontLeftDrive.setPower(drivePowers.frontLeft);
            backLeftDrive.setPower(drivePowers.backLeft);
            frontRightDrive.setPower(drivePowers.frontRight);
            backRightDrive.setPower(drivePowers.backRight);
        }
        public void SetMotorPowerBasic(double power) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backRightDrive.setPower(power);
        }
    }
