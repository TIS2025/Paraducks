package org.firstinspires.ftc.teamcode.AllCodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@TeleOp(name = "LED test")
@Config
public class LedTest extends LinearOpMode {

    // TODO ================================================== INSTANCES ============================================================================
    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OuttakeSubsystem outtake = new OuttakeSubsystem(robot);
        ElevatorSubsytem elevator = new ElevatorSubsytem(robot);



        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.x){ // Red off
                robot.LED1RED.setState(true);
            }
            if(gamepad1.y){ // Green off
                robot.LED1GREEN.setState(true);

            }
            if(gamepad1.a){ // Red On
                robot.LED1RED.setState(false);
            }
            if(gamepad1.b){ // Green On
                robot.LED1GREEN.setState(false);

            }

            if(gamepad1.left_bumper){
                robot.LED2RED.setState(true);
            }
            if(gamepad1.right_bumper){
                robot.LED2RED.setState(false);
            }
            if(gamepad1.left_trigger>0){
                robot.LED2GREEN.setState(true);
            }
            if(gamepad1.right_trigger>0){
                robot.LED2GREEN.setState(false);
            }

        }
    }

}
