package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.*;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Red Close 2+0", group = "Red")
public class Red_Close_TwoZero extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        GearRotationSubsystem gear = new GearRotationSubsystem(hardwareMap);
        Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        PresetSubsystem presets = new PresetSubsystem(claw, lift, gear);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Actions that run when Init
        camera.SPEED_GAIN = -0.02;   // Drive = Error * Gain
        camera.STRAFE_GAIN = -0.01;
        camera.TURN_GAIN = 0;
        claw.closeClaws();
        gear.wheelServo_Deactivated();
        camera.initAprilTag();
        camera.setManualExposure(2, 50);

        Pose2d RedCloseTwoZero_startPose = new Pose2d(62, 12, Math.toRadians(180));
        Pose2d RedCloseTwoZero_yellowScoringPose3 = new Pose2d(40, 29, Math.toRadians(270));
        Pose2d RedCloseTwoZero_yellowScoringPose2 = new Pose2d(28.5, 24, Math.toRadians(270));
        Pose2d RedCloseTwoZero_yellowScoringPose1 = new Pose2d(33.5, 10.5, Math.toRadians(270));
        Pose2d RedCloseTwoZero_yellowScoringOverridePose1 = new Pose2d(27, 45, Math.toRadians(270));
        Pose2d RedCloseTwoZero_yellowScoringOverridePose2 = new Pose2d(38, 45, Math.toRadians(270));
        Pose2d RedCloseTwoZero_yellowScoringOverridePose3 = new Pose2d(43.5, 45, Math.toRadians(270));
        Pose2d RedCloseTwoZero_parkingPose3 = new Pose2d(48, 55, Math.toRadians(270));
        Pose2d RedCloseTwoZero_parkingPose2 = new Pose2d(40, 55, Math.toRadians(270));
        Pose2d RedCloseTwoZero_parkingPose1 = new Pose2d(28, 55, Math.toRadians(270));

        //This action drives to the first tape line
        TrajectoryActionBuilder RedCloseTwoZero_purpleTAction1 = drive.actionBuilder(RedCloseTwoZero_startPose)
                .lineToX(55)
                .splineTo(new Vector2d(33, 12.5), Math.toRadians(270));
        Action RedCloseTwoZero_purpleAction1 = RedCloseTwoZero_purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder RedCloseTwoZero_purpleTAction2 = drive.actionBuilder(RedCloseTwoZero_startPose)
                .lineToX(55)
                .splineTo(new Vector2d(26, 24), Math.toRadians(270));
        Action RedCloseTwoZero_purpleAction2 = RedCloseTwoZero_purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder RedCloseTwoZero_purpleTAction3 = drive.actionBuilder(RedCloseTwoZero_startPose)
                .lineToX(55)
                .splineTo(new Vector2d(34, 31), Math.toRadians(270));
        Action RedCloseTwoZero_purpleAction3 = RedCloseTwoZero_purpleTAction3.build();

        //This action drives to the first backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringTAction1 = drive.actionBuilder(RedCloseTwoZero_yellowScoringPose1)
                .strafeTo(new Vector2d(27.5, 43));
        Action RedCloseTwoZero_yellowScoringAction1 = RedCloseTwoZero_yellowScoringTAction1.build();
        
        //This action drives to the second backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringTAction2 = drive.actionBuilder(RedCloseTwoZero_yellowScoringPose2)
                .strafeTo(new Vector2d(28.5,30))
                .strafeTo(new Vector2d(40, 45));
        Action RedCloseTwoZero_yellowScoringAction2 = RedCloseTwoZero_yellowScoringTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringTAction3 = drive.actionBuilder(RedCloseTwoZero_yellowScoringPose3)
                .strafeTo(new Vector2d(33.5,38))
                .strafeTo(new Vector2d(48, 45));

        Action RedCloseTwoZero_yellowScoringAction3 = RedCloseTwoZero_yellowScoringTAction3.build();

        //This action OVERRIDES camera and drives to the first backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringOverrideTAction1 = drive.actionBuilder(RedCloseTwoZero_yellowScoringOverridePose1)
                .strafeTo(new Vector2d(20.5, 55));
        Action RedCloseTwoZero_yellowScoringOverrideAction1 = RedCloseTwoZero_yellowScoringOverrideTAction1.build();

        //This action OVERRIDES camera and drives to the second backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringOverrideTAction2 = drive.actionBuilder(RedCloseTwoZero_yellowScoringOverridePose2)
                .strafeTo(new Vector2d(38, 55));
        Action RedCloseTwoZero_yellowScoringOverrideAction2 = RedCloseTwoZero_yellowScoringOverrideTAction2.build();

        //This action OVERRIDES camera and drives to the third backdrop section
        TrajectoryActionBuilder RedCloseTwoZero_yellowScoringOverrideTAction3 = drive.actionBuilder(RedCloseTwoZero_yellowScoringOverridePose3)
                .strafeTo(new Vector2d(43.5, 55));
        Action RedCloseTwoZero_yellowScoringOverrideAction3 = RedCloseTwoZero_yellowScoringOverrideTAction3.build();
        
        //This action drives to robot to the first parking zone
        TrajectoryActionBuilder RedCloseTwoZero_parkingTAction1 = drive.actionBuilder(RedCloseTwoZero_parkingPose1)
                .strafeTo(new Vector2d(38,43))
                .strafeTo((new Vector2d(69, 50)));
        Action RedCloseTwoZero_parkingAction1 = RedCloseTwoZero_parkingTAction1.build();

        //This action drives to robot to the second parking zone
        TrajectoryActionBuilder RedCloseTwoZero_parkingTAction2 = drive.actionBuilder(RedCloseTwoZero_parkingPose2)
                .strafeTo(new Vector2d(38,43))
                .strafeTo((new Vector2d(69, 50)));
        Action RedCloseTwoZero_parkingAction2 = RedCloseTwoZero_parkingTAction2.build();

        //This action drives to robot to the third parking zone
        TrajectoryActionBuilder RedCloseTwoZero_parkingTAction3 = drive.actionBuilder(RedCloseTwoZero_parkingPose3)
                .strafeTo(new Vector2d(38,43))
                .strafeTo((new Vector2d(69, 50)));
        Action RedCloseTwoZero_parkingAction3 = RedCloseTwoZero_parkingTAction3.build();


        //Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x

                //----------------------------1----------------------------\\
                if (blocks[i].x < 100 && blocks[i].id == 1 && blocks[i].y < 200) {

                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            RedCloseTwoZero_purpleAction1
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    RedCloseTwoZero_yellowScoringAction1
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(4);

                    /*if(!camera.targetFound){
                        Actions.runBlocking(
                                RedCloseTwoZero_yellowScoringOverrideAction1
                        );
                    }*/

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            RedCloseTwoZero_parkingAction1
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 1 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            RedCloseTwoZero_purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    RedCloseTwoZero_yellowScoringAction2
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(5);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                RedCloseTwoZero_yellowScoringOverrideAction2
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            RedCloseTwoZero_parkingAction2
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 210 && blocks[i].id == 1 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            RedCloseTwoZero_purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    RedCloseTwoZero_yellowScoringAction3
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(6);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                RedCloseTwoZero_yellowScoringOverrideAction3
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            RedCloseTwoZero_parkingAction3
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }
    }

}

