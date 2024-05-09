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

@Autonomous(name="Blue Close 2+2", group = "Blue")
public class Blue_Close_TwoTwo extends LinearOpMode {

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

        //---------------------------------------- Blue Close 2+2 ----------------------------------------------\\
        Pose2d BlueCloseTwoTwo_startPose = new Pose2d(-62, 12, 0);
        Pose2d BlueCloseTwoTwo_yellowScoringPose1 = new Pose2d(-40, 29, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_yellowScoringPose2 = new Pose2d(-28.5, 24, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_yellowScoringPose3 = new Pose2d(-33.5, 10.5, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_driveToWhitePose1 = new Pose2d(-42, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_driveToWhitePose2 = new Pose2d(-36, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_driveToWhitePose3 = new Pose2d(-30, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteTrussPose = new Pose2d(-44, 29, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteScoringPose = new Pose2d(-60,20, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_parkingPose1 = new Pose2d(-42, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_parkingPose2 = new Pose2d(-36, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_parkingPose3 = new Pose2d(-30, 55, Math.toRadians(270));


        //This action drives to the first tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction1 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-40, 29), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction1 = BlueCloseTwoTwo_purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction2 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-31, 24), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction2 = BlueCloseTwoTwo_purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction3 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction3 = BlueCloseTwoTwo_purpleTAction3.build();

        //This action drives to the first backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction1 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose1)
                .strafeTo(new Vector2d(-38, 45));
        Action BlueCloseTwoTwo_yellowScoringAction1 = BlueCloseTwoTwo_yellowScoringTAction1.build();


        //This action drives to the second backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction2 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose2)
                .strafeTo(new Vector2d(-32, 45));
        Action BlueCloseTwoTwo_yellowScoringAction2 = BlueCloseTwoTwo_yellowScoringTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction3 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose3)
                .strafeTo(new Vector2d(-33.5,15))
                .strafeTo(new Vector2d(-22, 45));
        Action BlueCloseTwoTwo_yellowScoringAction3 = BlueCloseTwoTwo_yellowScoringTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction1 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose1)
                .lineToY(43)
                .strafeTo(new Vector2d(-60, 20))
                .lineToY(-48)
                .strafeTo(new Vector2d(-36,-48));
        Action BlueCloseTwoTwo_driveToWhiteAction1 = BlueCloseTwoTwo_driveToWhiteTAction1.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction2 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose2)
                .lineToY(43)
                .strafeTo(new Vector2d(-60, 20))
                .lineToY(-48)
                .strafeTo(new Vector2d(-36,-48));
        Action BlueCloseTwoTwo_driveToWhiteAction2 = BlueCloseTwoTwo_driveToWhiteTAction2.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction3 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose3)
                .lineToY(43)
                .strafeTo(new Vector2d(-60, 20))
                .lineToY(-48)
                .strafeTo(new Vector2d(-36,-48));
        Action BlueCloseTwoTwo_driveToWhiteAction3 = BlueCloseTwoTwo_driveToWhiteTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_whiteTrussTAction = drive.actionBuilder(BlueCloseTwoTwo_whiteTrussPose)
                .strafeTo(new Vector2d(-60,-48))
                .lineToY(20);
        Action BlueCloseTwoTwo_whiteTrussAction = BlueCloseTwoTwo_whiteTrussTAction.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteScoringTAction = drive.actionBuilder(BlueCloseTwoTwo_whiteScoringPose)
                .strafeTo(new Vector2d(-32, 45));
        Action BlueCloseTwoTwo_whiteScoringAction = BlueCloseTwoTwo_whiteScoringTAction.build();


        //This action drives to robot to the parking zone
        TrajectoryActionBuilder BlueCloseTwoTwo_parkingTAction1 = drive.actionBuilder(BlueCloseTwoTwo_parkingPose1)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoTwo_parkingAction1 = BlueCloseTwoTwo_parkingTAction1.build();

        //This action drives to robot to the parking zone
        TrajectoryActionBuilder BlueCloseTwoTwo_parkingTAction2 = drive.actionBuilder(BlueCloseTwoTwo_parkingPose2)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoTwo_parkingAction2 = BlueCloseTwoTwo_parkingTAction2.build();

        //This action drives to robot to the parking zone
        TrajectoryActionBuilder BlueCloseTwoTwo_parkingTAction3 = drive.actionBuilder(BlueCloseTwoTwo_parkingPose3)
                .strafeTo(new Vector2d(-36,43))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoTwo_parkingAction3 = BlueCloseTwoTwo_parkingTAction3.build();



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
                                            BlueCloseTwoTwo_purpleAction1
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoTwo_yellowScoringAction1
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(1);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                presets.GroundPos(),
                                                    claw.openRClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction1
                                    ),
                                    presets.WhiteStack(),
                                    new SequentialAction(
                                            BlueCloseTwoTwo_whiteTrussAction,
                                            new ParallelAction(
                                                BlueCloseTwoTwo_whiteScoringAction,
                                                presets.ScoringPos()
                                            )
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoTwo_parkingAction2
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
                                            BlueCloseTwoTwo_purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoTwo_yellowScoringAction2
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openRClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction2
                                    ),
                                    presets.WhiteStack(),
                                    new SequentialAction(
                                            BlueCloseTwoTwo_whiteTrussAction,
                                            new ParallelAction(
                                                    BlueCloseTwoTwo_whiteScoringAction,
                                                    presets.ScoringPos()
                                            )
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoTwo_parkingAction2
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
                                            BlueCloseTwoTwo_purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoTwo_yellowScoringAction3
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(3);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openRClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction3
                                    ),
                                    presets.WhiteStack(),
                                    new SequentialAction(
                                            BlueCloseTwoTwo_whiteTrussAction,
                                            new ParallelAction(
                                                    BlueCloseTwoTwo_whiteScoringAction,
                                                    presets.ScoringPos()
                                            )
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            BlueCloseTwoTwo_parkingAction2
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }

    }


}

