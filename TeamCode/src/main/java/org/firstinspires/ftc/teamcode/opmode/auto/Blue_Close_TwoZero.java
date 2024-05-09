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

@Autonomous(name="Blue Close 2+0", group = "Blue")
public class Blue_Close_TwoZero extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        GearRotationSubsystem gear = new GearRotationSubsystem(hardwareMap);
        Pose2d beginPose = new Pose2d(-62, 12, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        PresetSubsystem presets = new PresetSubsystem(claw, lift, gear);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Actions that run when Init
        claw.closeClaws();
        gear.wheelServo_Deactivated();
        camera.initAprilTag();
        camera.setManualExposure(2, 250);

        //---------------------------------------- Blue Close 2+0 ----------------------------------------------\\
        Pose2d BlueCloseTwoZero_startPose = new Pose2d(-62, 12, 0);
        Pose2d BlueCloseTwoZero_yellowScoringPose1 = new Pose2d(-40, 29, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringPose2 = new Pose2d(-28.5, 24, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringPose3 = new Pose2d(-33.5, 10.5, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose1 = new Pose2d(-38, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose2 = new Pose2d(-31, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose3 = new Pose2d(-22, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_parkingPose1 = new Pose2d(-38, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_parkingPose2 = new Pose2d(-31, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_parkingPose3 = new Pose2d(-22, 55, Math.toRadians(270));


        //This action drives to the first tape line
        TrajectoryActionBuilder BlueCloseTwoZero_purpleTAction1 = drive.actionBuilder(BlueCloseTwoZero_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-40, 29), Math.toRadians(270));
        Action BlueCloseTwoZero_purpleAction1 = BlueCloseTwoZero_purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder BlueCloseTwoZero_purpleTAction2 = drive.actionBuilder(BlueCloseTwoZero_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-32, 21), Math.toRadians(270));//-31, 24
        Action BlueCloseTwoZero_purpleAction2 = BlueCloseTwoZero_purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder BlueCloseTwoZero_purpleTAction3 = drive.actionBuilder(BlueCloseTwoZero_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270));
        Action BlueCloseTwoZero_purpleAction3 = BlueCloseTwoZero_purpleTAction3.build();

        //This action drives to the first backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringTAction1 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringPose1)
                .strafeTo(new Vector2d(-38, 45))
                .turnTo(Math.toRadians(270));
        Action BlueCloseTwoZero_yellowScoringAction1 = BlueCloseTwoZero_yellowScoringTAction1.build();

        //This action drives to the second backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringTAction2 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringPose2)
                .strafeTo(new Vector2d(-31, 45))
                .turnTo(Math.toRadians(270));
        Action BlueCloseTwoZero_yellowScoringAction2 = BlueCloseTwoZero_yellowScoringTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringTAction3 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringPose3)
                .strafeTo(new Vector2d(-33.5,15))
                .strafeTo(new Vector2d(-22, 45))
                .turnTo(Math.toRadians(270));
        Action BlueCloseTwoZero_yellowScoringAction3 = BlueCloseTwoZero_yellowScoringTAction3.build();

        //This action OVERRIDES camera and drives to the first backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringOverrideTAction1 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringOverridePose1)
                .strafeTo(new Vector2d(-38, 55));
        Action BlueCloseTwoZero_yellowScoringOverrideAction1 = BlueCloseTwoZero_yellowScoringOverrideTAction1.build();

        //This action OVERRIDES camera and drives to the second backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringOverrideTAction2 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringOverridePose2)
                .strafeTo(new Vector2d(-31, 55));
        Action BlueCloseTwoZero_yellowScoringOverrideAction2 = BlueCloseTwoZero_yellowScoringOverrideTAction2.build();

        //This action OVERRIDES camera and drives to the third backdrop section
        TrajectoryActionBuilder BlueCloseTwoZero_yellowScoringOverrideTAction3 = drive.actionBuilder(BlueCloseTwoZero_yellowScoringOverridePose3)
                .strafeTo(new Vector2d(-22, 55));
        Action BlueCloseTwoZero_yellowScoringOverrideAction3 = BlueCloseTwoZero_yellowScoringOverrideTAction3.build();

        //This action drives to robot to the first parking zone
        TrajectoryActionBuilder BlueCloseTwoZero_parkingTAction1 = drive.actionBuilder(BlueCloseTwoZero_parkingPose1)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoZero_parkingAction1 = BlueCloseTwoZero_parkingTAction1.build();

        //This action drives to robot to the second parking zone
        TrajectoryActionBuilder BlueCloseTwoZero_parkingTAction2 = drive.actionBuilder(BlueCloseTwoZero_parkingPose2)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoZero_parkingAction2 = BlueCloseTwoZero_parkingTAction2.build();

        //This action drives to robot to the third parking zone
        TrajectoryActionBuilder BlueCloseTwoZero_parkingTAction3 = drive.actionBuilder(BlueCloseTwoZero_parkingPose3)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoZero_parkingAction3 = BlueCloseTwoZero_parkingTAction3.build();



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
                if (blocks[i].x < 100 && blocks[i].id == 2 && blocks[i].y < 200) {

                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            BlueCloseTwoZero_purpleAction1
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoZero_yellowScoringAction1
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(1);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                BlueCloseTwoZero_yellowScoringOverrideAction1
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoZero_parkingAction1
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            BlueCloseTwoZero_purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoZero_yellowScoringAction2
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                BlueCloseTwoZero_yellowScoringOverrideAction2
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoZero_parkingAction2
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            BlueCloseTwoZero_purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoZero_yellowScoringAction3
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(3);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                BlueCloseTwoZero_yellowScoringOverrideAction3
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoZero_parkingAction3
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }

    }


}

