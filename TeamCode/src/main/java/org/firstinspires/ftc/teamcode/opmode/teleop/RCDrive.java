package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.GearRotationSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.PresetSubsystem;
@Disabled
@TeleOp
public class RCDrive extends OpMode {
    ClawSubsystem.claw claw = new ClawSubsystem.claw(hardwareMap);
    LiftSubsystem.lift lift = new LiftSubsystem.lift(hardwareMap);
    GearRotationSubsystem.gear gear = new GearRotationSubsystem.gear(hardwareMap);
    PresetSubsystem.presets presets = new PresetSubsystem.presets(hardwareMap);

    @Override
    public void init() {
        gear.resetGear();
        lift.resetLift();
        presets.InitPos();
    }

    @Override
    public void loop() {

    }

}
