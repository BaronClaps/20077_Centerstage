
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

package org.firstinspires.ftc.teamcode.opmode.archived.recent;

import androidx.annotation.NonNull;

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

import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.DriveTrajectoriesSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;
@Autonomous(name="OldBlueCloseCamera")

public class BlueCloseCamera extends LinearOpMode{

    /* Hardware Names */
    private final int READ_PERIOD = 1;
    private ElapsedTime aprilTagTime = new ElapsedTime();
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

    /* April Tag Movement Values */
    double DESIRED_DISTANCE = 1.5; // In Inches
    final double SPEED_GAIN  = -0.02;   // Drive = Error * Gain
    final double STRAFE_GAIN = -0.01;
    double TURN_GAIN = 0.01;
    double MAX_AUTO_SPEED = 0.9;
    double MAX_AUTO_STRAFE= 0.9;
    double MAX_AUTO_TURN  = 0.65;

    /* Camera Initialization */
    private static final boolean USE_WEBCAM = true;
    private static int DESIRED_TAG_ID = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int aprilTagDecimation = 3;

    @Override public void runOpMode() {

        /* Initialize RoadRunner */
        Pose2d beginPose = new Pose2d(-62, 12, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Pose2d scoringPose1 = new Pose2d(-42, 55, Math.toRadians(270));
        Pose2d scoringPose2 = new Pose2d(-36, 55, Math.toRadians(270));
        Pose2d scoringPose3 = new Pose2d(-30, 55, Math.toRadians(270));


        /* Initialize the hardware variables */
        lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wheelServo = hardwareMap.get(Servo.class, "WheelServo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rB");

        /* Initialize the April Tag Process */
        boolean targetFound = false;
        double forward = 0;
        double strafe = 0;
        double turn = 0;
        targetFound = false;
        desiredTag = null;
        initAprilTag();
        if (USE_WEBCAM) setManualExposure(2, 250);

        /* Initialize Servos */
        clawL.setPosition(.33);
        clawR.setPosition(.38);
        wheelServo.setPosition(.85);

        /* Initialize Motors */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        gear.setDirection(DcMotor.Direction.REVERSE);
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Initialize the HuskyLens */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        /* Wait until Play */
        telemetry.update();
        waitForStart();

        /* Code Runs when Played */
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            if (!rateLimit.hasExpired()) {
                continue;
            }
            aprilTagTime.reset();
            rateLimit.reset();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            /*------------------------------------------------------------ HuskyLens ------------------------------------------------------------*/
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                telemetry.addData("location?", blocks[i].x);
                //-----------------------------------------------------------1-----------------------------------------------------------\\
                if (blocks[i].x < 100 && blocks[i].id == 2 && blocks[i].y < 200) {

                    DESIRED_TAG_ID = 1;
                    TURN_GAIN =  0;

                    //----------------------------------- Start Roadrunner ----------------------------------\\
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)

                                    /* Start Position */
                                    .stopAndAdd(closeR())
                                    .stopAndAdd(closeL())
                                    .stopAndAdd(up())
                                    .stopAndAdd(gearStartPos())



                                    /* Score Purple */
                                    .lineToX(-55)
                                    .splineTo(new Vector2d(-40, 29), Math.toRadians(269.99))
                                    .waitSeconds(.1)
                                    .stopAndAdd(openL())
                                    .stopAndAdd(closeR())

                                    /* Drive to Camera Location */
                                    .waitSeconds(.25)
                                    .turnTo((Math.toRadians(270)))
                                    .stopAndAdd(flipToScore_1stCycle_Outside())
                                    .stopAndAdd(liftExtend_Cycle1_Yellow())
                                    .strafeTo(new Vector2d(-38, 45))
                                    .waitSeconds(.1)
                                    .build());

                    //----------------------------------- April Tag Alignment ----------------------------------\\
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            } else {
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    if (targetFound) {
                        // Determine heading, range and Yaw (tag image rotation) error
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    }

                    aprilTagTime.reset();

                    while (aprilTagTime.seconds() <= 1.5) {
                        moveRobot(forward,strafe,turn);
                    }

                    telemetry.addData("time",aprilTagTime);

                    //----------------------------------- Resume Roadrunner ----------------------------------\\
                    Actions.runBlocking(
                            drive.actionBuilder(scoringPose1)

                                    /* Score Yellow */
                                    .stopAndAdd(openR())
                                    .waitSeconds(.25)

                                    /* Park and Reset for Teleop */
                                    .lineToY(43)
                                    .strafeTo((new Vector2d(-67, 50)))
                                    .waitSeconds(.1)
                                    .stopAndAdd(up())
                                    .waitSeconds(.1)
                                    .stopAndAdd(gearEndPos())
                                    .waitSeconds(.1)
                                    .stopAndAdd(liftRetract_Cycle1_Yellow())
                                    .waitSeconds(.25)
                                    // .lineToY(63)
                                    .build());
                    sleep(400000);

                }

                //-----------------------------------------------------------2-----------------------------------------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200)
                {
                    TURN_GAIN   =  0;
                    DESIRED_TAG_ID = 2;

                    //----------------------------------- Start Roadrunner ----------------------------------\\
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)

                                    /* Start Position */
                                    .stopAndAdd(closeR())
                                    .stopAndAdd(closeL())
                                    .stopAndAdd(up())
                                    .stopAndAdd(gearStartPos())
                                    .waitSeconds(.1)

                                    /* Score Purple */
                                    .splineTo(new Vector2d(-28.5,24), Math.toRadians(279.99))
                                    .waitSeconds(.1)
                                    .stopAndAdd(openL())
                                    .stopAndAdd(closeR())

                                    /* Drive to Camera Location */
                                    .waitSeconds(.25)
                                    .strafeTo(new Vector2d(-33,45))
                                    .stopAndAdd(flipToScore_1stCycle_Outside())
                                    .stopAndAdd(liftExtend_Cycle1_Yellow())
                                    .turnTo(Math.toRadians(270))
                                    .build());

                    //----------------------------------- April Tag Alignment ----------------------------------\\
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            } else {
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    if (targetFound) {
                        // Determine heading, range and Yaw (tag image rotation) error
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    }

                    aprilTagTime.reset();

                    while (aprilTagTime.seconds() <= 1.5) {
                        moveRobot(forward,strafe,turn);
                    }

                    telemetry.addData("time",aprilTagTime);

                    //----------------------------------- Resume Roadrunner ----------------------------------\\
                    Actions.runBlocking(
                            drive.actionBuilder(scoringPose2)

                                    /* Score Yellow */
                                    .stopAndAdd(openR())
                                    .waitSeconds(.25)

                                    /* Park and Reset for Teleop */
                                    .lineToY(43)
                                    .strafeTo((new Vector2d(-67, 50)))
                                    .waitSeconds(.1)
                                    .stopAndAdd(up())
                                    .waitSeconds(.1)
                                    .stopAndAdd(gearEndPos())
                                    .waitSeconds(.1)
                                    .stopAndAdd(liftRetract_Cycle1_Yellow())
                                    .waitSeconds(.25)
                                    //  .lineToY(63)
                                    .build());
                    sleep(400000);
                }

                //-----------------------------------------------------------3-----------------------------------------------------------\\
                if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200)
                {
                    DESIRED_TAG_ID = 3;
                    TURN_GAIN   =  0;

                    //----------------------------------- Start Roadrunner ----------------------------------\\

                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)

                                    /* Start Position */
                                    .stopAndAdd(closeR())
                                    .stopAndAdd(closeL())
                                    .stopAndAdd(up())
                                    .stopAndAdd(gearStartPos())
                                    .waitSeconds(.1)

                                    /* Score Purple */
                                    .lineToX(-55)
                                    .waitSeconds(.1)
                                    .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270))
                                    .waitSeconds(.1)
                                    .stopAndAdd(openL())
                                    .lineToY(15)

                                    /* Drive to Camera Location */
                                    .stopAndAdd(flipToScore_1stCycle_Inside())
                                    .stopAndAdd(liftExtend_Cycle1_Yellow())
                                    .stopAndAdd(closeL())
                                    .strafeTo(new Vector2d(-22, 45))
                                    .build());

                    //----------------------------------- April Tag Alignment ----------------------------------\\
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            } else {
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    if (targetFound) {
                        // Determine heading, range and Yaw (tag image rotation) error
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    }

                    aprilTagTime.reset();

                    while (aprilTagTime.seconds() <= 1.5) {
                        moveRobot(forward,strafe,turn);
                    }

                    telemetry.addData("time",aprilTagTime);

                    //----------------------------------- Resume Roadrunner ----------------------------------\\
                    Actions.runBlocking(
                            drive.actionBuilder(scoringPose3)

                                    /* Score Yellow */
                                    .stopAndAdd(openR())
                                    .waitSeconds(.25)

                                    /* Park and Reset for Teleop */
                                    .lineToY(43)
                                    .strafeTo((new Vector2d(-67, 50)))
                                    .waitSeconds(.1)
                                    .stopAndAdd(up())
                                    .waitSeconds(.1)
                                    .stopAndAdd(gearEndPos())
                                    .waitSeconds(.1)
                                    .stopAndAdd(liftRetract_Cycle1_Yellow())
                                    .waitSeconds(.25)
                                    //  .lineToY(63)
                                    .build());
                    sleep(400000);
                }

            }
        }
    }

    /*------------------------------------------------------------ April Tag Functions ------------------------------------------------------------*/
    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(aprilTagDecimation);

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
    public void setManualExposure(int exposureMS, int gain) {
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
        if (!isStopRequested()) {
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

    public void moveRobot(double x, double y, double yaw) {
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
    }



    /*------------------------------------------------------------ Actions ------------------------------------------------------------*/
    public Action gearStartPos() {
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

    public Action gearEndPos() {
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
                gear.setPower(0);
                return false;
            }
        };
    }

    public Action liftExtend_Cycle1_Yellow() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-600);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }

    public Action liftRetract_Cycle1_Yellow() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(600);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }

    public Action flipToScore_1stCycle_Outside() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.23);
                gear.setTargetPosition(750);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.4);
                return false;
            }
        };
    }

    public Action flipToScore_1stCycle_Inside() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.23);
                gear.setTargetPosition(725);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.4);
                return false;
            }
        };
    }

    public Action flipToScore_2ndCycle() {
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

    /* Wheel Servo Actions */
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

    public Action closeL(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawL.setPosition(0.33); return false;
            }
        };
    }
    public Action closeR(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawR.setPosition(0.37); return false;
            }
        };
    }


    public Action openL(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawL.setPosition(0.42); return false;
            }
        };
    }
    public Action openR(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawR.setPosition(0.28); return false;
            }
        };
    }

    public Action up(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.815); return false;
            }
        };
    }

    public Action pivotPickUp(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.87); return false;
            }
        };
    }

}