package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    //Todo ========================================================= Outake Actuators ===================================================================

    public  ServoImplEx Gripper=null;
    public  ServoImplEx Wrist=null;
    public  ServoImplEx Arm=null;

    public ServoImplEx Shoulder = null;
    public ServoImplEx RackExtend = null;

    public ServoImplEx RackExtensionLeft = null;
    public ServoImplEx RackExtensionRight = null;


    //Todo ========================================================= Elevator Actuators ===================================================================
    public DcMotorEx LeftElevator=null;
    public DcMotorEx RightElevator=null;


    // TODO ====================================================== Hanger Actuators =====================================================================
    public ServoImplEx Hanger1 = null;
    public ServoImplEx Hanger2 = null;

    public DcMotorEx TiltMotor=null;


    //Todo ========================================================= Color Sensor ===================================================================
    public RevColorSensorV3 SensorColor1;

    // TODO ========================================================= Rack Analog ================================================================

    public AnalogInput rackAnalog1;



    //Todo ========================================================= Robot Setup ===================================================================
    private static RobotHardware instance = null;    // ref variable to use robot hardware
    public boolean enabled;                          //boolean to return instance if robot is enabled.

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public HardwareMap hardwareMap;

    //Todo init() for hardware map
    //Call this method inside auto and teleop classes to instantly hardware map all actuators.
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;


        //Todo ========================================================= Map Outtake Actuators ===================================================================
        Gripper = hardwareMap.get(ServoImplEx.class, "grip");
        Wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        Arm = hardwareMap.get(ServoImplEx.class, "arm");
        Shoulder = hardwareMap.get(ServoImplEx.class, "shoulder");
        RackExtend = hardwareMap.get(ServoImplEx.class, "rack");

        RackExtensionLeft = hardwareMap.get(ServoImplEx.class, "leftRack");
        RackExtensionRight = hardwareMap.get(ServoImplEx.class, "rightRack");


        //Todo ========================================================= Elevator Actuators ===================================================================

        RightElevator = hardwareMap.get(DcMotorEx.class, "elevatorRight");
        LeftElevator = hardwareMap.get(DcMotorEx.class, "elevatorLeft");



        // TODO ========================================================= Hanger Actuators ==============================================================
        Hanger1 = hardwareMap.get(ServoImplEx.class, "hanger1");
        Hanger2 = hardwareMap.get(ServoImplEx.class, "hanger2");

        TiltMotor = hardwareMap.get(DcMotorEx.class, "Tilt");

        //Todo ========================================================= Color Sensor ===================================================================
        SensorColor1 = hardwareMap.get(RevColorSensorV3.class, "cs1");

        // TODO ========================================================== Rack Analog ===================================================================
        rackAnalog1 = hardwareMap.get(AnalogInput.class, "rackAnalog1");

        //Todo ========================================================= Setting Mode for all the Actuators ===================================================================
        LeftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

}
