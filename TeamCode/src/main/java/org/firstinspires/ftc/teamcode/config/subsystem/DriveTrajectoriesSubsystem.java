package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;

public class DriveTrajectoriesSubsystem {
    public static class driveTrajectories {
        private Pose2d beginPose;
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


    }
}
