package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



public final class Constants {

  public static final double MAX_SPEED = 5.450;
  //swerve modulleri tanımlama matematik:
  public static final class ElevatorConstants {
      public static final int leftMotorID = 31;
      public static final int rightMotorID = 32;

      public static final double kS = 0.0;
      public static final double kG = 0.075;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double maxSpeed = 4.0;
      public static final double maxAccel = 8.0;

      public static final double rampRate = 0.25;

      public static final double specialConstant = 0.04; // Ballpark estimate
  }
  public static final class ShooterConstants {
      public static final int shooterID = 35;
  }
  public static final class IntakeConstants {
    public static final int intakeLeftID = 33;
    public static final int intakeRightID = 34;
  }
  public static final class yukari{
    public static final int solelevator = 31;
    public static final int sagelevator = 32;
    public static final int climbmotor = 30;
    public static final int shooter = 35;
    public static final int shootertaci = 34;
    //--------------------------------------------------
//ELEVATOR:
    //pıd değerler(örnek değerler; değişecek)
    public final static double kP = 0.1; //.068
    public final static double kI = 0.0;
    public final static double kD = 0.00; //.005

    public final static double kS = 0.0;
    public final static double kG = 0.075; //.07
    public final static double kV = 0.0;
    public final static double kA = 0.0;

    public static final double pulleyDiameterMeters = 0.05 ; // 0.05metre
    public static final double pulleyCircumference = Math.PI * pulleyDiameterMeters;
    public static final double gearReduction = 6.8;  // Örneğin, 10:1 oran
    public static final double encoderConversionFactor = pulleyCircumference / gearReduction;
    // Bu, motor enkoder sayısını (dönüş) metreye çevirecektir.
    // Maksimum ve minimum elevator yüksekliği (metre cinsinden)
    public static final double minHeight = 0.0;
    public static final double maxHeight = 2.0;  // 2 metre

  }
  public static final class Swerve {

    public static final double stickDeadband = 0.09;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.585;
    // Distance between right and left wheels
    public static final double wheelBase = 0.585;
    // Distance between front and back wheelsxx
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 30;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.001;
    public static final double angleKI = 0.000;
    public static final double angleKD = 0.00;
    public static final double angleKFF = 0.000;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0001;
    public static final double driveKI = 0.00005;
    public static final double driveKD = 0.0005;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio ;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.82; // meters per second

    // public static final double maxSpeed = 5880.0 / 60.0 * 0.1633 * 0.1016 *Math.PI; // 5.1 meters per second
    //public static final double maxAngularVelocity = 11.5; 
    public static final double maxAngularVelocity = maxSpeed/Math.hypot(trackWidth/2,wheelBase/2); //11.65

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    public static final double kTranslationVarianceThreshold = 0.1;  // Örneğin, 0.1 metre
    public static final double kAngleVarianceThreshold = 5.0;

  }
}
