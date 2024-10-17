package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    // TODO==========================================================  FOR NAV X ====================================================
    public static boolean IS_CUSTOMIMU=false;


    // TODO==========================================================  Elevator Value ====================================================
    public static int incVal= 1;
    public static int home=0;
    public static  int highBasket= 4800;
    public static  int lowBasket= 3200;
    public static int specimenHanged = 900;
    public static int specimenPicked = 814;
    public static  int highRung= 1841;
    public static int lowrung= 1426;

    // TODO==========================================================  Hanging Value ====================================================
    public static  int hangpos = 0;
    public static int hang = 0;

    // TODO==========================================================  Shoulder Servo Value ====================================================
    public static double shoulderInit=0.34;


                // ================================== Sample Pick and Drop ================================================
    public static double shoulderSamplePrePick= 0.6317;
    public static double shoulderSamplePick= 0.5806;
    public static double shoulderSamplePick1= 0.5556;
    public static double speed= 0.75;
    public static double shoulderSamplePreDrop= 0;
    public static double shoulderSampleDrop= 0.2506;

                // ================================== Sequence Pick and Drop ================================================

    public static double shoulderSpecimenPrePick= 0;
    public static double shoulderSpecimenPick= 0.14;//0.1339;
    public static double shoulderSpecimenPreDrop= 0;
    public static double shoulderSpecimenDrop= 0.2428;


    // TODO==========================================================  Left / Right Gripper Servo Value ====================================================

    //Gripper
    public static double gripperOpen=0.32;
    public static double gripperSafe=0.4556;
    public static double gripperClose=0.6828;


    // TODO==========================================================  Rotate Servo Value ====================================================

    public static double rackInit=0.88;
    public static double rackSamplePick=  0.1844;
    public static double rackSampleDrop= 0.88;
    public static double rackSpecimenPick=0.55;
    public static double rackSpecimenDrop= 0.1844;


    // TODO =================================================================  ARM Value  ====================================================

    public static double armInit= 0.455;
    public static double armSamplePrePick=0;
    public static double armSamplePick= 0.4367;
    public static double armSamplePreDrop=0;
    public static double armSampleDrop=0;
    public static double armSequencePrePick=0;
    public static double armSpecimenPick=  0.12;//0.1217;
    public static double armSpecimenpreDrop=0;
    public static double armSpecimenDrop=  0.5878;

    // TODO ================================================================ OPENcv ====================================================================
    public static Location SIDE = Location.CLOSE;
    public static Location ALLIANCE = Location.BLUE;


    // TODO ============================================================== Wrist Value ===================================================================
    public  static double wristInit = 0.6806;
    public  static double wristSamplePick = 0.715;
    public  static double Wrist90 =  0.4339;
    public  static double wristSampleDrop = 0.7067;

    public  static double wristSpecimenPick = 0.1306;//0.7067;
    public  static double wristSpecimenDrop = 0.7017;

    // TODO ============================================================ Rack Extension =============================================================
    public static double rackExtensionRightSamplePIck = 0.8706;
    public static double rackExtensionRightInit = 0.895;
    public static double rackExtensionRightSpecimenPIck = 0.975;//0.8706;
    public static double rackExtensionRightSampleDrop = 0.8706;
    public static double rackExtensionRightSpecimenDrop = 0.93;

//
//    // TODO ========================================================= CServo ======================================================================
//
//    public static double InitTime = 0;
//    public static double SpecimenDropTime = 2;
//    public static double SpecimenPickTime = 2;
//    public static double SamplePickTime = 2;
//    public static double SampleDropTime = 2;


}





