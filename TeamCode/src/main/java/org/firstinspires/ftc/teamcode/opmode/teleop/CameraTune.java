package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="CameraTune")
public class CameraTune extends LinearOpMode {

    /* Motor Intialization */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    /* April Tag Movement Values */
    double DESIRED_DISTANCE = 3; // In Inches
    double SPEED_GAIN = -0.025;   // Drive = Error * Gain
    double STRAFE_GAIN = -0.01;
    double TURN_GAIN = 0;
    double MAX_AUTO_SPEED = 0.9;
    double MAX_AUTO_STRAFE = 0.9;
    double MAX_AUTO_TURN = 0.65;
    double forward;
    double strafe;
    double turn;
    public boolean targetFound = false;


    /* Camera Initialization */
    private ElapsedTime aprilTagTime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int aprilTagDecimation = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rB");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        //----------------------------Main-Code----------------------------\\
        initAprilTag();
        setManualExposure(2, 250);

        waitForStart();
        while (opModeIsActive()) {

            //----------------------------Mecanum-Drive-Code----------------------------\\
            if (gamepad1.right_bumper) {
                double max;
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                double lFPower = axial + lateral + yaw;
                double rFPower = axial - lateral - yaw;
                double lBPower = axial - lateral + yaw;
                double rBPower = axial + lateral - yaw;
                max = Math.max(Math.abs(lFPower), Math.abs(rFPower));
                max = Math.max(max, Math.abs(lBPower));
                max = Math.max(max, Math.abs(rBPower));
                if (max > 1.0) {
                    lFPower /= max;
                    rFPower /= max;
                    lBPower /= max;
                    rBPower /= max;
                }

                    leftFrontDrive.setPower(0.8 * lFPower);
                    rightFrontDrive.setPower(0.8 * rFPower);
                    leftBackDrive.setPower(0.8 * lBPower);
                    rightBackDrive.setPower(0.8 * rBPower);

            }

            if (!gamepad1.right_bumper) {
                alignToTag(2);
            }
                telemetry.update();


        }


    }

    public void moveRobot ( double x, double y, double yaw){
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    public void alignToTag ( int DESIRED_TAG_ID){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                }
            } else {

            }
        }

        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            if(rangeError < 0){
                SPEED_GAIN = 0.025;
            } else {
                SPEED_GAIN = -0.025;
            }

            if(yawError < 0){
                STRAFE_GAIN = 0.01;;
            } else {
                STRAFE_GAIN = -0.01;;
            }

            if(headingError < 0){
                TURN_GAIN = -0.001;
            } else {
                TURN_GAIN = 0.001;
            }

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            aprilTagTime.reset();

            moveRobot(forward, strafe, turn);
            }
        }


    public void initAprilTag () {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                .addProcessor(aprilTag)
                .build();

    }

    public void setManualExposure ( int exposureMS, int gain){
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            }
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

    }
}
