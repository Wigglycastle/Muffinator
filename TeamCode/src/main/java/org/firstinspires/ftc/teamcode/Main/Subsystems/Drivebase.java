package org.firstinspires.ftc.teamcode.Main.Subsystems;

// The basics

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private boolean speedBool;

    private final GoBildaPinpointDriver pinpoint;
    double speedFactor = 1;

    public Drivebase(HardwareMap hardwareMap) {

        // For heading
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        //TODO: Get heading data from end of auto, set to temp value
        pinpoint.setHeading(90, AngleUnit.DEGREES);

        // Define motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Set Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }
    public void ProcessInput(GamepadEx gamepad) {

        // Get heading data from pinpoint
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        // Set speed factor, when A is pressed the robot goes into a fine adjustment mode, on a toggle
        if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            if (speedBool) {
                speedBool = false;
                speedFactor = 1;
            }
            else {
                speedBool = true;
                speedFactor = 0.5;
            }
        }

        // Reset Pos
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
        }

        // Grab inputs from the Gamepad and apply speed factor
        double x    =  -gamepad.getLeftY() * speedFactor;
        double y    =  gamepad.getLeftX() * speedFactor;
        double rx   =  gamepad.getRightX() * speedFactor;

        // Rotate input vector by the negative heading
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Calculate, normalize, multiply by speed factor, and send power to motors
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontDrive.setPower(( rotY + rotX + rx) / denom);
        leftBackDrive.setPower(( rotY - rotX + rx) / denom);
        rightFrontDrive.setPower(( rotY - rotX - rx) / denom);
        rightBackDrive.setPower(( rotY + rotX - rx) / denom);
        }
        public boolean GetSlowMode() {
        return speedBool;
        }
    }
