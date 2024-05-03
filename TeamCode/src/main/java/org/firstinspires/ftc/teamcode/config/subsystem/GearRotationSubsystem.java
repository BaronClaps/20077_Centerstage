package org.firstinspires.ftc.teamcode.config.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GearRotationSubsystem {
    public static class gear {
        private DcMotorEx gear;

        public gear(HardwareMap hardwareMap) {
            gear = hardwareMap.get(DcMotorEx.class, "gear");
            gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            gear.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //------------------------------Ground Position------------------------------//
        public Action groundGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    gear.setTargetPosition(0);
                    gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    gear.setPower(0.5);
                    return false;
                }
            };
        }

        //------------------------------Scoring Position------------------------------//
        public Action scoringGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    gear.setTargetPosition(735);
                    gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    gear.setPower(0.5);
                    return false;
                }
            };
        }

        //------------------------------Start Position-------------------------------//
        public Action startGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    gear.setTargetPosition(-500);
                    gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    gear.setPower(0.5);
                    return false;
                }
            };
        }

        //------------------------------End Position-------------------------------//
        public Action endGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    gear.setTargetPosition(-500);
                    gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    gear.setPower(0.5);
                    return false;
                }
            };
        }

        //------------------------------ Wait for Gear -------------------------------//
        public Action waitForGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return gear.isBusy();
                }
            };
        }

        //------------------------------ Stop Gear -------------------------------//
        public Action stopGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    gear.setPower(0);
                    return false;
                }
            };
        }

    }
}
