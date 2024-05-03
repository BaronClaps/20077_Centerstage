package org.firstinspires.ftc.teamcode.config.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
    public static class lift {
        private DcMotorEx lift;

        public lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "gear");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        //------------------------------ Lift Extend ------------------------------//
        public Action liftExtend_Scoring() {
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

        //------------------------------ Lift Retract ------------------------------//
        public Action liftRetract_Scoring() {
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

        //------------------------------ Wait for Gear -------------------------------//
        public Action waitForLift() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return lift.isBusy();
                }
            };
        }

        //------------------------------ Stop Gear -------------------------------//
        public Action stopGear() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setPower(0);
                    return false;
                }
            };
        }


    }
}
