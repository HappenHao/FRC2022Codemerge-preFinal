
package frc.robot;


import edu.wpi.first.wpilibj.DoubleSolenoid;

public final class Constants {
    public static final class Contorller {
        public static final int joystickID = 1;
        public static final int xboxID = 0;
        public static final int JoystickVersionButtin = 2;

    }

    public static final class DriverConstant {
        public static final int left_front = 33;
        public static final int left_back = 34;
        public static final int right_front = 31;
        public static final int right_back = 32;
        public static final int pigeno_ID = 3;

        public static final double gearboxratio = 10.71;
        public static final double wheeldiameter = 0.152; // Unit: meter

        public static final double Rpm2Mps = (1 / gearboxratio) * wheeldiameter * Math.PI / 60;
        public static final double a=gearboxratio/(wheeldiameter* Math.PI);

        public static final double DriverRun_kp = 0.02;

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static Gains kGains_GyroTurn = new Gains(0.004, 0.00001, 0, 0, 0, 0);

    }

    public static final class ShooterConstant {
        public static final int left_shooter = 24; //
        public static final int right_shooter = 25;
        public static final int rotate = 15;
        public static final int elevation = 16;
        public static final int launch = 14;

        public static final int rotate_left_limit = 30;
        public static final int rotate_right_limit = -40;
        public static final int elevation_limit = 45;

        public static final int rotate_init_angle = -99;
        public static final int elevation_init_angle = 50;

        public static final double rotation_ratio   = 20*296/29;     // 296/29 = 10.2069
        //20 is the radio of Planetary gearbox
        //the gear on out_Hex gear is 29T
        //the gear on ratation is 296T
        public static final double elevation_ratio= 20 * 19.33;     // 464/23 = 20.17
        //20 is the radio of Planetary gearbox
        //the gear on HEX is 23T
        //the gear on elevation is about 464T
        public static final double flywheel_ratio =36/24; 
        public static final double flywheel_diameter=0.1016 ;//meter  36T/24T=1.5  4in=101.6mm=0.1016m 
        public static final double flywheel_girth = flywheel_diameter*Math.PI;
        public static final double flywheel_up10ms_mps = 2048 *flywheel_ratio*flywheel_girth;

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static Gains kGains_Velocit  = new Gains( 0.1, 0.00001 , 0, 1023.0/20660.0,  300,  1.00);

        //* motor_pitch.set(setang(0.01, 0.000015, 0,degtoenc(tarpitch)-encoder_pitch.getPosition())); 
        public final static Gains kGains_Pitch = new Gains(0.01, 0.000015, 0, 0, 0, 0);
        public final static Gains kGains_Spin = new Gains(0.02, 0.000, 0, 0, 0, 0);   //kp=0.017


    }

    public static final class DeliverConstant {
        public static final int intake = 11; //
        public static final int front = 12; //
        public static final int back = 13; //

        public static final Double DeliverInvent = 1.0;
    }

    public static final class controlConstant {
        public static final int PDP = 1; //
        public static final int PCM = 0; //
    }

    //
    public static final class PneumaticConstant {
        public static final int k_IntakeU = 0;
        public static final int k_IntakeD = 1;
    }

    //
    public static final class PneumaticStatues {
        public static final DoubleSolenoid.Value kIntakeUp = DoubleSolenoid.Value.kForward;//
        public static final DoubleSolenoid.Value kIntakeDown = DoubleSolenoid.Value.kReverse;//
    }

    //
    public static final class SensorConstant {
        public static final int rotate_digital = 2;
        public static final int pitch_digital = 1;
        public static final int shootExit_digital = 0;
        public static final int colorSensor_IIC = 0x52;
        public static final int Addressable_LED=0;
        public static final int Addressable_LED_Number=100;
    }

    public static final class PhotonVisionConstant {
        public static final String Front_Cam = "FrontCam";
        public static final String Shooter_Cam = "ShooterCam";
        public static final String Team = "empty";

        double camDiagFOV = 170.0; // degrees - assume wide-angle camera
        double camPitch = 45; // degrees
        double camHeightOffGround = 0.5; // meters
        double maxLEDRange = 20; // meters
        int camResolutionWidth = 640; // pixels
        int camResolutionHeight = 480; // pixels
        double minTargetArea = 10; // square pixels


        public static final double Camera_height = 0.685;                   //meter of Camera Height
        public static final double Target_height = 2.67;                    //meter of Target
        public static final double Camera_pitch = 29 * Math.PI / 180;     //Cam pitch Angle(degree)
        public static final double dis_error = 0.45;

        public final static Gains kGains_rotateVersion = new Gains(0.1, 0, 0, 0, 0, 1.00);

    }

}
