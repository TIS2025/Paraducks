package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    // TODO==========================================================  FOR NAV X ====================================================
    public static boolean IS_CUSTOMIMU=false;


    // TODO==========================================================  Elevator Value ====================================================
    public static int incVal= 1;
    public static int home=0;
    public static  int highBasket= 4400;
    public static  int lowBasket= 2100;
    public static int specimenHanged = 450;//980;//980;
    public static int specimenPicked = 1932;
    public static  int highRung= 1450;//1850;//1841;
    public static int lowrung= 1226;

    // TODO==========================================================  Hanging Value ====================================================
    public static  int hangpos = 0;
    public static int hang = 0;

    // TODO==========================================================  Shoulder Servo Value ====================================================
    public static double shoulderInit=0.34;


                // ================================== Sample Pick and Drop ================================================
    public static double shoulderSamplePrePick= 0.6317;
    public static double shoulderSamplePick= 0.59;//0.6028;//0.5806;
    public static double shoulderLimeSamplePick= 0.54;//0.6028;//0.5806;
    public static double AutoshoulderSamplePick= 0.54;//0.5806;
    public static double shoulderSamplePick1= 0.5556;
    public static double speed= 0.75;
    public static double shoulderSamplePreDrop= 0;
    public static double shoulderSampleDrop= 0.2589;//0.1489;//0.2506;
                // ================================== Sequence Pick and Drop ================================================

    public static double shoulderSpecimenPrePick= 0;
    public static double shoulderSpecimenPick= 0.12;//0.13;//0.1339;
    public static double shoulderSpecimenPreDrop= 0;
    public static double shoulderSpecimenDrop= 0.35;//0.3478;//0.3617;`
    public static double AutoshoulderSpecimenDrop= 0.2617;//0.3617;

    // TODO==========================================================  Left / Right Gripper Servo Value ====================================================

    //Gripper
    public static double gripperOpen=0.38;
    public static double gripperLimeOpen=0.30;
    public static double gripperSafe=0.4556;
    public static double gripperClose=0.6828;
    public static double gripperOpenMore=0.2306;




    // TODO==========================================================  Rack Servo Value ====================================================

    public static double rackVal = 0.12;
    public static double rackInit=rackVal;//0.4778;//0.88;
    public static double rackSamplePick=  rackVal;//0.1844;
    public static double rackSampleDrop= rackVal + 0.3617;//0.7117;//0.35;//0.88;
    public static double rackSpecimenPick=rackVal;//0.35;//0.53;
    public static double rackSpecimenDrop= rackVal-0.29;//0.06;//0.1844;
    public static double autoRackSampleDrop= rackVal+ 0.323;//0.673;//0.8;





    // TODO =================================================================  ARM Value  ====================================================

    public static double armInit= 0.455;
    public static double armSamplePrePick=0;
    public static double armSamplePick= 0.4367;
    public static double limeSamplePick= 0.2;
    public static double armSamplePreDrop=0;
    public static double armSampleDrop= 0;//0.1106;//0;
    public static double armSequencePrePick=0;
    public static double armSpecimenPick=  0.12;//0.1217;
    public static double armSpecimenpreDrop=0;
    public static double armSpecimenDrop=  0.45;//0.5489;//0.4;//0.4489;//0.5878;//0.5667;//0.5878;
    public static double AutoarmSpecimenDrop=  0.5878;//0.4;//0.4489;//0.5878;//0.5667;//0.5878;

    // TODO ================================================================ OPENcv ====================================================================
    public static Location SIDE = Location.CLOSE;
    public static Location ALLIANCE = Location.BLUE;


    // TODO ============================================================== Wrist Value ===================================================================
    public  static double wristInit = 0.695;// 0.6806;
    public  static double wristSamplePick = 0.715;
    public  static double Wrist90 =  0.4339;
    public  static double WristLeftDiagonal =  0.3106;
    public  static double WristRightDiagonal =  0.6359;//0.555;
    public  static double wristSampleDrop = 0.6878;//0.7156;//0.7067;

    public  static double wristSpecimenPick = 0.1294;//0;//0.7067;
    public  static double wristSpecimenDrop = 0.6817;//0.1306;//0.6956;//0.6978;//0.7017;

    // TODO ============================================================ Rack Extension =============================================================
    public static double rackExtensionRightSamplePIck = 0.8706;
    public static double rackExtensionRightInit = 0.9139; //0.895;
    public static double rackExtensionRightSpecimenPIck = 0.975;
    public static double rackExtensionRightSampleDrop = 0.8706;
    public static double rackExtensionRightSpecimenDrop = 0.93;
    public static double rackExtensionMaxPos = 0.3767;
    public static double rackExtensionMidPos = 0.55;


//    // TODO ========================================================= CServo ======================================================================
//
//    public static double InitTime = 0;
//    public static double SpecimenDropTime = 2;
//    public static double SpecimenPickTime = 2;
//    public static double SamplePickTime = 2;
//    public static double SampleDropTime = 2;


}





