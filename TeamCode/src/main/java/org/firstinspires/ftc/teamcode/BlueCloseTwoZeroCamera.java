
/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
import android.graphics.Bitmap;
import android.graphics.Canvas;

import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;


/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="BlueCloseTwoZeroCamera")

public class BlueCloseTwoZeroCamera extends LinearOpMode{

    private final int READ_PERIOD = 1;
    private ElapsedTime aprilTagTime = new ElapsedTime();

    public int pixelspot;
    private DcMotor  leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor  rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor lift = null;
    private DcMotor gear = null;
    private Servo pivot = null;
    private Servo clawL = null;
    private Servo clawR = null;
    private Servo wheelServo = null;
    private HuskyLens huskyLens;
     //----------------April Tag Detection Values--------------//
    double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
     final double SPEED_GAIN  =  -0.015 ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  -0.01 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.0175  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.45;   //  Clip the turn speed to this max value (adjust for your robot)

    //-------------------Camera Initialization---------------------------//
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag





    @Override public void runOpMode()
    {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double forward = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rB");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        Pose2d beginPose = new Pose2d(-60, 12, 0); //Pose2d beginPose = new Pose2d(60, -30, Math.toRadians(180)); for red
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Pose2d scoringPose1 = new Pose2d(-42, 55, -Math.PI / 2);
        Pose2d scoringPose2 = new Pose2d(-36, 55, -Math.PI / 2);
        Pose2d scoringPose3 = new Pose2d(-30, 55, -Math.PI / 2);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wheelServo = hardwareMap.get(Servo.class, "WheelServo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        clawL.setPosition(.33);
        clawR.setPosition(.38);
        wheelServo.setPosition(.85);


        gear.setDirection(DcMotor.Direction.REVERSE);
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();

        if (!huskyLens.knock())
        {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag


            // Tell the driver what we see, and what to do.
            if (targetFound)
            {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                if (!rateLimit.hasExpired())
                {
                    continue;
                }

                rateLimit.reset();
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Block count", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());
                    telemetry.addData("location?", blocks[i].x);
                    //TODO ensure your x values of the husky lens are appropriate to the desired areas
                    //----------------------------1----------------------------\\
                    if (blocks[i].x < 100 && blocks[i].id == 2 && blocks[i].y < 200) {
                        DESIRED_TAG_ID = 1;
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        /* Start Position */
                                        .stopAndAdd(drive.closeR())
                                        .stopAndAdd(drive.closeL())
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.5)
                                        .stopAndAdd(gearStartPos())
                                        .waitSeconds(.1)

                                        /* Score Purple */
                                        .lineToX(-55)
                                        .waitSeconds(.1)
                                        .splineTo(new Vector2d(-37, 29), -(Math.PI / 2))
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.openL())
                                        .stopAndAdd(drive.closeR())

                                        /* Drive to Camera Location */
                                        .waitSeconds(.25)
                                        .stopAndAdd(flipToScore_1stCycle())
                                        .stopAndAdd(liftExtend_Cycle1_Yellow())
                                        .strafeTo(new Vector2d(-36, 45))
                                        .build());

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections)
                        {
                            // Look to see if we have size info on this tag.
                            if (detection.metadata != null)
                            {
                                //  Check to see if we want to track towards this tag.
                                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))
                                {
                                    // Yes, we want to use this tag.
                                    targetFound = true;
                                    desiredTag = detection;
                                    break;  // don't look any further.
                                } else {
                                    // This tag is in the library, but we do not want to track it right now.
                                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                                }
                            } else {
                                // This tag is NOT in the library, so we don't have enough information to track to it.
                                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                            }
                        }

                        if (targetFound)
                        {

                            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            double headingError = desiredTag.ftcPose.bearing;
                            double yawError = desiredTag.ftcPose.yaw;

                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        }

                        aprilTagTime.reset();

                        while (aprilTagTime.seconds() <= 1) {
                        moveRobot(forward,strafe,turn);
                        }

                        Actions.runBlocking(
                                drive.actionBuilder(scoringPose1)
                                        /* Score Yellow */
                                        .stopAndAdd(drive.openR())//score yellow
                                        .waitSeconds(.25)

                                        /* Park and Reset for Teleop */
                                        .lineToY(43)
                                        .strafeTo((new Vector2d(-67.5, 50)))
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.1)
                                        .stopAndAdd(gearEndPos())
                                        .waitSeconds(.1)
                                        .stopAndAdd(liftRetract_Cycle1_Yellow())
                                        .waitSeconds(.25)
                                        .lineToY(63)
                                        .build());
                        sleep(400000);

                    }
                    //----------------------------2----------------------------\\
                    if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200)
                    {
                        DESIRED_TAG_ID = 2;
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        /* Start Position */
                                        .stopAndAdd(drive.closeR())
                                        .stopAndAdd(drive.closeL())
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.5)
                                        .stopAndAdd(gearStartPos())
                                        .waitSeconds(.1)

                                        /* Score Purple */
                                        .lineToX(-33)
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.openL())
                                        .stopAndAdd(drive.closeR())

                                        /* Drive to Camera Location */
                                        .waitSeconds(.25)
                                        .turn(-Math.PI/2)
                                        .waitSeconds(.1)
                                        .stopAndAdd(flipToScore_1stCycle())
                                        .stopAndAdd(liftExtend_Cycle1_Yellow())
                                        .strafeTo(new Vector2d(-36, 45))
                                        .build());

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections) {
                            // Look to see if we have size info on this tag.
                            if (detection.metadata != null) {
                                //  Check to see if we want to track towards this tag.
                                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                    // Yes, we want to use this tag.
                                    targetFound = true;
                                    desiredTag = detection;
                                    break;  // don't look any further.
                                } else {
                                    // This tag is in the library, but we do not want to track it right now.
                                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                                }
                            } else {
                                // This tag is NOT in the library, so we don't have enough information to track to it.
                                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                            }
                        }

                        if (targetFound) {

                            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            double headingError = desiredTag.ftcPose.bearing;
                            double yawError = desiredTag.ftcPose.yaw;

                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        }


                        aprilTagTime.reset();

                        while (aprilTagTime.seconds() <= 1)
                        {
                            moveRobot(forward,strafe,turn);
                        }

                        Actions.runBlocking(
                                drive.actionBuilder(scoringPose2)
                                        /* Score Yellow */
                                        .stopAndAdd(drive.openR())//score yellow
                                        .waitSeconds(.25)

                                        /* Park and Reset for Teleop */
                                        .lineToY(43)
                                        .strafeTo((new Vector2d(-67.5, 50)))
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.1)
                                        .stopAndAdd(gearEndPos())
                                        .waitSeconds(.1)
                                        .stopAndAdd(liftRetract_Cycle1_Yellow())
                                        .waitSeconds(.25)
                                        .lineToY(63)
                                        .build());
                        sleep(400000);
                    }

                    //----------------------------3----------------------------\\
                    if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200)
                    {
                        DESIRED_TAG_ID = 3;
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        /* Start Position */
                                        .stopAndAdd(drive.closeR())
                                        .stopAndAdd(drive.closeL())
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.5)
                                        .stopAndAdd(gearStartPos())
                                        .waitSeconds(.1)

                                        /* Score Purple */
                                        .lineToX(-55)
                                        .waitSeconds(.1)
                                        .splineTo(new Vector2d(-36, 6.5), -(2 * Math.PI) / (3))
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.openL())
                                        .stopAndAdd(drive.closeR())

                                        /* Drive to Camera Location */
                                        .waitSeconds(.25)
                                        .stopAndAdd(flipToScore_1stCycle())
                                        .stopAndAdd(liftExtend_Cycle1_Yellow())
                                        .strafeTo(new Vector2d(-36, 45))
                                        .build());

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections)
                        {
                            // Look to see if we have size info on this tag.
                            if (detection.metadata != null)
                            {
                                //  Check to see if we want to track towards this tag.
                                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))
                                {
                                    // Yes, we want to use this tag.
                                    targetFound = true;
                                    desiredTag = detection;
                                    break;  // don't look any further.
                                } else {
                                    // This tag is in the library, but we do not want to track it right now.
                                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                                }
                            } else {
                                // This tag is NOT in the library, so we don't have enough information to track to it.
                                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                            }
                        }

                        if (targetFound)
                        {

                            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            double headingError = desiredTag.ftcPose.bearing;
                            double yawError = desiredTag.ftcPose.yaw;

                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        }


                        aprilTagTime.reset();

                        while (aprilTagTime.seconds() <= 1)
                        {
                            moveRobot(forward,strafe,turn);
                        }

                        Actions.runBlocking(
                                drive.actionBuilder(scoringPose3)
                                        /* Score Yellow */
                                        .stopAndAdd(drive.openR())//score yellow
                                        .waitSeconds(.25)

                                        /* Park and Reset for Teleop */
                                        .lineToY(43)
                                        .strafeTo((new Vector2d(-67.5, 50)))
                                        .waitSeconds(.1)
                                        .stopAndAdd(drive.up())
                                        .waitSeconds(.1)
                                        .stopAndAdd(gearEndPos())
                                        .waitSeconds(.1)
                                        .stopAndAdd(liftRetract_Cycle1_Yellow())
                                        .waitSeconds(.25)
                                        .lineToY(63)
                                        .build());
                        sleep(400000);
                    }

                }
            }
        }

     public void initAprilTag()
     {
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
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        public void setManualExposure(int exposureMS, int gain)
        {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }

        public Action moveRobot(double x, double y, double yaw)
        {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    double leftFrontPower    =  x -y -yaw;
                    double rightFrontPower   =  x +y +yaw;
                    double leftBackPower     =  x +y -yaw;
                    double rightBackPower    =  x -y +yaw;

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
                    return false;
                }
            };
        }


    public Action gearStartPos()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-500);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.25);

                while (gear.isBusy()) {
                    sleep(25);
                }

                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setPower(0);
                return false;
            }
        };
    }

    public Action gearEndPos()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-505);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.4);

                while (gear.isBusy()) {
                    sleep(25);
                }

                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setPower(0); return false;
            }
        };
    }

    /* Close Side Extensions */
    public Action liftExtend_Cycle1_Yellow()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-650);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }
    public Action liftRetract_Cycle1_Yellow()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(650);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }






    public Action flipToScore_1stCycle()
    {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.28);
                gear.setTargetPosition(700);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.22);

                return false;
            }
        };
    }


    public Action flipToScore_2ndCycle(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gear.setTargetPosition(300);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.333);
                pivot.setPosition(0.75); //was .74


                return false;
            }
        };
    }


    public Action wheelServo_Up_Z1() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.641); //bigger # = lower


                return false;
            }
        };
    }
    public Action wheelServo_Up_Z2() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.6); //bigger # = lower | ~ 0.03 per pixel


                return false;
            }
        };
    }
    public Action wheelServo_Down() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.85);

                return false;
            }
        };
    }
    public Action wheelServo_Up_Z3() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.621); //bigger # = lower | ~ 0.03 per pixel

                return false;
            }
        };
    }
    public Action wheelServo_Up_Z1_First() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.63); //bigger # = lower


                return false;
            }
        };
    }
    public Action wheelServo_Up_Z2_First() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.597); //bigger # = lower | ~ 0.03 per pixel


                return false;
            }
        };
    }
    public Action wheelServo_Up_Z3_First() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wheelServo.setPosition(0.621); //bigger # = lower | ~ 0.03 per pixel


                return false;
            }
        };
    }







}
