package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;

public class DriveTrajectoriesSubsystem {
    public static class driveTrajectories {
        Pose2d beginPose;
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        public driveTrajectories(HardwareMap hardwareMap) {
        }

        Action purpleAction1;
        Action purpleAction2;
        Action purpleAction3;
        Action yellowAction1;
        Action yellowAction2;
        Action yellowAction3;
        Action parkingAction1;
        Action parkingAction2;
        Action parkingAction3;


        public void AutoSelection(String Color, String Side, int WhitePixels) {
            switch(Color) {

                case "Blue":
                    switch(Side) {

                        case "Close":
                            Pose2d beginPose = new Pose2d(-62, 12, 0);
                            switch(WhitePixels) {
                                case 0: // 2+0

                                    //This action drives to the first tape line
                                    TrajectoryActionBuilder purpleTAction1 = drive.actionBuilder(beginPose)
                                            .lineToX(-55)
                                            .splineTo(new Vector2d(-40, 29), Math.toRadians(270));

                                    purpleAction1 = purpleTAction1.build();

                                    //This action drives to the second tape line
                                    TrajectoryActionBuilder purpleTAction2 = drive.actionBuilder(beginPose)
                                            .lineToX(-55)
                                            .splineTo(new Vector2d(-28.5,24), Math.toRadians(270));

                                    purpleAction2 = purpleTAction2.build();

                                    //This action drives to the third tape line
                                    TrajectoryActionBuilder purpleTAction3 = drive.actionBuilder(beginPose)
                                            .lineToX(-55)
                                            .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270));

                                    purpleAction3 = purpleTAction3.build();

                                    //This action drives to the first backdrop section


                                    TrajectoryActionBuilder yellowTAction1 = purpleTAction1.fresh()
                                            .turnTo((Math.toRadians(270)))
                                            .strafeTo(new Vector2d(-38, 45))
                                            .turnTo((Math.toRadians(270)));

                                    yellowAction1 = yellowTAction1.build();


                                    //This action drives to the second backdrop section
                                    TrajectoryActionBuilder yellowTAction2 = purpleTAction2.fresh()
                                            .turnTo(Math.toRadians(270))
                                            .strafeTo(new Vector2d(-33,45))
                                            .turnTo(Math.toRadians(270));

                                    yellowAction2 = yellowTAction2.build();

                                    //This action drives to the third backdrop section
                                    TrajectoryActionBuilder yellowTAction3 = purpleTAction3.fresh()
                                            .lineToY(15)
                                            .turnTo((Math.toRadians(270)))
                                            .strafeTo(new Vector2d(-22, 45))
                                            .turnTo((Math.toRadians(270)));

                                    yellowAction3 = yellowTAction3.build();


                                    //dfgjkhjkdfghjkhjkdfgdfghjkh fdhklfghjkl need diff poses if camera
                                    //This action drives to robot to the first parking zone
                                    TrajectoryActionBuilder parkingTAction1 = yellowTAction1.fresh()
                                            .lineToY(43)
                                            .strafeTo((new Vector2d(-67, 50)));

                                    parkingAction1 = parkingTAction1.build();

                                    //This action drives to robot to the second parking zone
                                    TrajectoryActionBuilder parkingTAction2 = yellowTAction2.fresh()
                                            .lineToY(43)
                                            .strafeTo((new Vector2d(-67, 50)));

                                    parkingAction2 = parkingTAction2.build();

                                    //This action drives to robot to the third parking zone
                                    TrajectoryActionBuilder parkingTAction3 = yellowTAction3.fresh()
                                            .lineToY(43)
                                            .strafeTo((new Vector2d(-67, 50)));

                                    parkingAction3 = parkingTAction3.build();

                                    break;

                                case 2: // 2+2
                                    break;

                                case 4: // 2+4
                                    break;
                            }
                        break;

                        case "Far":
                            switch(WhitePixels) {
                                case 0:
                                   break;

                                case 2:
                                    break;

                                case 4:
                                    break;
                            }
                        break;

                    }
                break;

                case "Red":
                    switch(Side) {

                        case "Close":
                            switch(WhitePixels) {
                                case 0:

                                    break;

                                case 2:
                                    break;

                                case 4:
                                    break;
                            }
                            break;

                        case "Far":
                            switch(WhitePixels) {
                                case 0:
                                    break;

                                case 2:
                                    break;

                                case 4:
                                    break;
                            }
                            break;

                    }
                    break;
            }

        }

    }
}
