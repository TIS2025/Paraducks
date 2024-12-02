package org.firstinspires.ftc.teamcode.AllCodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public int ThresholdColor=1000;
    public int ThresholdDistance=15;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        while(opModeInInit()){
            robot.Shoulder.setPosition(Globals.shoulderSamplePick);
        }
        waitForStart();

        while (opModeIsActive()){
            if(((robot.SensorColor1.red()>=ThresholdColor || robot.SensorColor1.blue()>=ThresholdColor || robot.SensorColor1.green()>=ThresholdColor ) && robot.SensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                    && ((robot.SensorColor1.red()>=ThresholdColor || robot.SensorColor1.blue()>=ThresholdColor || robot.SensorColor1.green()>=ThresholdColor) && robot.SensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
                telemetry.addLine("Got You");
            }


            telemetry.addData("Red Value", robot.SensorColor1.red());
            telemetry.addData("Blue Value", robot.SensorColor1.blue());
            telemetry.addData("Green Value", robot.SensorColor1.green());

            telemetry.addData("Distance Value", robot.SensorColor1.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
