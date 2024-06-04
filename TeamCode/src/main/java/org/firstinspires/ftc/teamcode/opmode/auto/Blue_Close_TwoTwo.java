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

        camera.SPEED_GAIN = -0.02;   // Drive = Error * Gain
        camera.STRAFE_GAIN = 0.01;
        camera.TURN_GAIN = 0;

        // Actions that run when Init
        claw.closeClaws();
        gear.wheelServo_Deactivated();
        camera.initAprilTag();
        camera.setManualExposure(2, 50);

        //---------------------------------------- Blue Close 2+2 ----------------------------------------------\\
        Pose2d BlueCloseTwoTwo_startPose = new Pose2d(-62, 12, 0);
        Pose2d BlueCloseTwoTwo_yellowScoringPose1 = new Pose2d(-40, 27.5, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_yellowScoringPose2 = new Pose2d(-31, 22, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_yellowScoringPose3 = new Pose2d(-33.5, 10.5, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose1 = new Pose2d(-38, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose2 = new Pose2d(-31, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoZero_yellowScoringOverridePose3 = new Pose2d(-28, 45, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_driveToWhitePose1 = new Pose2d(-37, 55, Math.toRadians(267));//-38
        Pose2d BlueCloseTwoTwo_driveToWhitePose2 = new Pose2d(-31, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_driveToWhitePose3 = new Pose2d(-24, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteTrussPose = new Pose2d(-35,-36.5, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteScoringPose1 = new Pose2d(-64, 36, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteScoringPose2 = new Pose2d(-60, 36, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_whiteScoringPose3 = new Pose2d(-60, 36, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_parkingPose1 = new Pose2d(-40.5, 52.75, Math.toRadians(90));
        /*Pose2d BlueCloseTwoTwo_parkingPose1 = new Pose2d(-41, 54, Math.toRadians(90));
        Pose2d BlueCloseTwoTwo_parkingPose2 = new Pose2d(-36, 55, Math.toRadians(270));
        Pose2d BlueCloseTwoTwo_parkingPose3 = new Pose2d(-30, 55, Math.toRadians(270));*/
        // Hi hope you have a nice day!

        //This action drives to the first tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction1 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-40, 27.5), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction1 = BlueCloseTwoTwo_purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction2 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-31, 22), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction2 = BlueCloseTwoTwo_purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder BlueCloseTwoTwo_purpleTAction3 = drive.actionBuilder(BlueCloseTwoTwo_startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270));
        Action BlueCloseTwoTwo_purpleAction3 = BlueCloseTwoTwo_purpleTAction3.build();

        //This action drives to the first backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction1 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose1)
                .strafeTo(new Vector2d(-37, 45));
        Action BlueCloseTwoTwo_yellowScoringAction1 = BlueCloseTwoTwo_yellowScoringTAction1.build();

        //This action drives to the second backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction2 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose2)
                .strafeTo(new Vector2d(-31, 45));
        Action BlueCloseTwoTwo_yellowScoringAction2 = BlueCloseTwoTwo_yellowScoringTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder BlueCloseTwoTwo_yellowScoringTAction3 = drive.actionBuilder(BlueCloseTwoTwo_yellowScoringPose3)
                .strafeTo(new Vector2d(-33.5,15))
                .strafeTo(new Vector2d(-24, 45));
        Action BlueCloseTwoTwo_yellowScoringAction3 = BlueCloseTwoTwo_yellowScoringTAction3.build();

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
                .strafeTo(new Vector2d(-28, 55));
        Action BlueCloseTwoZero_yellowScoringOverrideAction3 = BlueCloseTwoZero_yellowScoringOverrideTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction1 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose1)
                .strafeTo(new Vector2d(-41,47))
                .strafeToConstantHeading(new Vector2d(-63, 36))
                .strafeTo(new Vector2d(-63,-36))
                .strafeToConstantHeading(new Vector2d(-35,-36.5));
        Action BlueCloseTwoTwo_driveToWhiteAction1 = BlueCloseTwoTwo_driveToWhiteTAction1.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction2 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose2)
                //.strafeTo(new Vector2d(-31,43))
                .strafeTo(new Vector2d(-41,47))
                .strafeToLinearHeading(new Vector2d(-63, 36), Math.toRadians(267))//267
                .strafeTo(new Vector2d(-63,-36))
                .strafeToLinearHeading((new Vector2d(-35,-36.5)), Math.toRadians(270));
        Action BlueCloseTwoTwo_driveToWhiteAction2 = BlueCloseTwoTwo_driveToWhiteTAction2.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_driveToWhiteTAction3 = drive.actionBuilder(BlueCloseTwoTwo_driveToWhitePose3)
                //.strafeTo(new Vector2d(-31,43))
                .strafeTo(new Vector2d(-24,47))
                .strafeToLinearHeading(new Vector2d(-63, 36), Math.toRadians(267))
                .strafeTo(new Vector2d(-63,-36))
                .strafeToLinearHeading(new Vector2d(-35,-36.5), Math.toRadians(270));
        Action BlueCloseTwoTwo_driveToWhiteAction3 = BlueCloseTwoTwo_driveToWhiteTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder BlueCloseTwoTwo_whiteTrussTAction1 = drive.actionBuilder(BlueCloseTwoTwo_whiteTrussPose)
                .strafeToConstantHeading(new Vector2d(-64,-36))
                .strafeToConstantHeading(new Vector2d(-62,-12))
                .strafeToConstantHeading(new Vector2d(-62, 36));
        Action BlueCloseTwoTwo_whiteTrussAction1 = BlueCloseTwoTwo_whiteTrussTAction1.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteTrussTAction2 = drive.actionBuilder(BlueCloseTwoTwo_whiteTrussPose)
                .strafeToLinearHeading((new Vector2d(-62.5,-48)),Math.toRadians(270))
                .strafeTo(new Vector2d(-60,-12))
                .strafeTo(new Vector2d(-60, 36));
        Action BlueCloseTwoTwo_whiteTrussAction2 = BlueCloseTwoTwo_whiteTrussTAction2.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteTrussTAction3 = drive.actionBuilder(BlueCloseTwoTwo_whiteTrussPose)
                .strafeToLinearHeading((new Vector2d(-62.5,-48)),Math.toRadians(270))
                .strafeTo(new Vector2d(-60,-12))
                .strafeTo(new Vector2d(-60, 36));
        Action BlueCloseTwoTwo_whiteTrussAction3 = BlueCloseTwoTwo_whiteTrussTAction3.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteScoringTAction1 = drive.actionBuilder(BlueCloseTwoTwo_whiteScoringPose1)
                .splineTo(new Vector2d(-40.5, 52.75), Math.toRadians(90));
        Action BlueCloseTwoTwo_whiteScoringAction1 = BlueCloseTwoTwo_whiteScoringTAction1.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteScoringTAction2 = drive.actionBuilder(BlueCloseTwoTwo_whiteScoringPose2)
                .splineTo(new Vector2d(-40.5, 52.75), Math.toRadians(90));
        Action BlueCloseTwoTwo_whiteScoringAction2 = BlueCloseTwoTwo_whiteScoringTAction2.build();

        TrajectoryActionBuilder BlueCloseTwoTwo_whiteScoringTAction3 = drive.actionBuilder(BlueCloseTwoTwo_whiteScoringPose3)
                .splineTo(new Vector2d(-40.5, 52.75), Math.toRadians(90));
        Action BlueCloseTwoTwo_whiteScoringAction3 = BlueCloseTwoTwo_whiteScoringTAction3.build();


        //This action drives to robot to the parking zone
        TrajectoryActionBuilder BlueCloseTwoTwo_parkingTAction = drive.actionBuilder(BlueCloseTwoTwo_parkingPose1)
                .strafeTo(new Vector2d(-38,40))
                .strafeTo((new Vector2d(-67, 50)));
        Action BlueCloseTwoTwo_parkingAction = BlueCloseTwoTwo_parkingTAction.build();
        /*
        //This action drives to robot to the parking zone
        TrajectoryActionBuilder BlueCloseTwoTwo_parkingTAction1 = drive.actionBuilder(BlueCloseTwoTwo_parkingPose1)
                .strafeTo(new Vector2d(-38,40))
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
*/


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
                                    new SleepAction(0.2),
                                    claw.openLClaw(),
                                    new SleepAction(0.35),
                                    new ParallelAction(
                                            new ParallelAction(
                                                    presets.ScoringPos(),
                                                    BlueCloseTwoTwo_yellowScoringAction1
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
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction1
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            BlueCloseTwoTwo_whiteTrussAction1,
                                            new ParallelAction(
                                                BlueCloseTwoTwo_whiteScoringAction1,
                                                presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            BlueCloseTwoTwo_parkingAction,
                                            presets.WhiteGroundPos()
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
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction2
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            BlueCloseTwoTwo_whiteTrussAction2,
                                            new ParallelAction(
                                                    BlueCloseTwoTwo_whiteScoringAction2,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.25),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            BlueCloseTwoTwo_parkingAction,
                                            presets.WhiteGroundPos()
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
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            BlueCloseTwoTwo_driveToWhiteAction3
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            BlueCloseTwoTwo_whiteTrussAction3,
                                            new ParallelAction(
                                                    BlueCloseTwoTwo_whiteScoringAction3,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            BlueCloseTwoTwo_parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }

    }


}

