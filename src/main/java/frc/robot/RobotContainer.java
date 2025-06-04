// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

//import static edu.wpi.first.units.Units.Newton;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.print.event.PrintServiceAttributeEvent;
import javax.swing.colorchooser.DefaultColorSelectionModel;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.SnakeCaseStrategy;
import com.fasterxml.jackson.databind.deser.impl.NullsAsEmptyProvider;
import com.fasterxml.jackson.databind.jsontype.BasicPolymorphicTypeValidator.NameMatcher;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
public class RobotContainer {

    private static final double VISION_DES_ANGLE_deg = 0.0;
    private static final double VISION_TURN_kP = 0.008;
    private static final double VISION_STRAFE_kP = 0.01;
    private static final double VISION_DES_RANGE_m = 0.5;
    // private static final double ANGLE_TOLERANCE = 1.0;   // Açısal tolerans (derece)
    // private static final double POSITION_TOLERANCE = 0.05; 
      /* Controllers */
    private final Joystick driver = new Joystick(0); 
    private final Joystick driver2 = new Joystick(1);
  
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis =  2;
  
  
    private final JoystickButton robotCentric =
    new JoystickButton(driver, 1);    
  
    private final JoystickButton zeroGyro =
    new JoystickButton(driver, 3);
  
    private final JoystickButton xLock = 
    new JoystickButton(driver, 6);
  
    private final JoystickButton climb_In = 
    new JoystickButton(driver,7);
  
    private final JoystickButton climb_Bas =
    new JoystickButton(driver, 8 );
  
    private final JoystickButton shooter_Tukur =
    new JoystickButton(driver2, 1);
  
    private final JoystickButton shooter_IcineAl =
    new JoystickButton(driver2, 2);
  
    private final JoystickButton shooter_Aci_Yukari = 
    new JoystickButton(driver2, 4);
  
    private final JoystickButton shooter_Aci_Asagi = 
    new JoystickButton(driver2, 3);
  
    private final JoystickButton elevator_l4Button = 
    new JoystickButton(driver2, 7);
  
    private final JoystickButton elevator_l3Button = 
    new JoystickButton(driver2, 9);
  
    private final JoystickButton elevator_l2Button =
    new JoystickButton(driver2, 11);
  
    private final JoystickButton elevator_kapat = 
    new JoystickButton(driver2, 8);
  
    private final JoystickButton shooter_duzelt = 
    new JoystickButton(driver2 , 10);

    private final JoystickButton manualElevator =
    new JoystickButton(driver2, 5);
   
    private final JoystickButton aimattarget = 
    new JoystickButton(driver, 5);
  
    // private final JoystickButton photon =
    // new JoystickButton(driver, 9);
  
    //private final JoystickButton driveToPos =
    // new JoystickButton(driver, 10);
  
  
    public final Swerve s_Swerve = new Swerve();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final ClimberSubsystem climb = new ClimberSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final Vision vision = new Vision(s_Swerve::addVisionMeasurement);

    //public final Vision vision = new Vision(s_Swerve::getPose, new edu.wpi.first.wpilibj.smartdashboard.Field2d());
  
    //public final shooter_vur shooter_tukur = new shooter_vur(s_yukari);
    //public final shooter_al shooter_icineal = new shooter_al(s_yukari);
  
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(s_Swerve.getSwerveDrive(), () -> -driver.getY(), () -> -driver.getX())
    .withControllerRotationAxis(() -> -driver.getZ())
    .deadband(Constants.Swerve.stickDeadband)
    .scaleTranslation(1)
    .allianceRelativeControl(true);
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3)).headingWhile(true);
  
    Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
    Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);
    // A chooser for autonomous commands
  
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      NamedCommands.registerCommand("ShooterIntake", new ShooterCommand(intake, 1.5, false));
      NamedCommands.registerCommand("ShooterSpit", new ShooterCommand(intake, 1.5, true));

//      NamedCommands.registerCommand("elevator_l4", elevator_otonom(s_yukari, -892, -42, -0.5));
//      NamedCommands.registerCommand("elevator_l3", elevator_otonom(s_yukari, -456, -42, -0.5));
//      NamedCommands.registerCommand("elevator_l2", elevator_otonom(s_yukari, -135, -37,-0.5));
//      NamedCommands.registerCommand("erene_duzelt", elevator_otonom(s_yukari, -92, -11.5, -0.4));
//      NamedCommands.registerCommand("elevator_kapa", elevator_kapat(s_yukari, -5.0, 0.2));
//      NamedCommands.registerCommand("Shooter_duzelt", elevator_otonom(s_yukari, -0, -20,-0.3));
      //NamedCommands.registerCommand("alignApril", alignWithAprilTag(s_Swerve, camera, 21));
  
  
      DriverStation.silenceJoystickConnectionWarning(true);
      s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
      configureButtonBindings();
      m_chooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(m_chooser);
      SmartDashboard.putString("pose", s_Swerve.getPose().toString());
      s_Swerve.resetModulesToAbsolute();
    
    
    }


    //private Command alignWithAprilTag(Swerve s_Swerve, PhotonCamera camera, int tagID) {
    //}

//    }
    //  private Command decSpeed (Swerve s_Swerve, double maxSpeed){
    //    return new RunCommand(() -> s_Swerve, s_Swerve.setMaximumSpeed -= 1);
    //  }
  
    private void configureButtonBindings() {
  
      try {
      // Vision alt sisteminizin getCamera() metodunu ekleyin.
      // aimattarget.whileTrue(new AlignWithAprilTag(s_Swerve, camera,21));
      
//      photon.whileTrue(new RunCommand(() -> photonvision(s_Swerve, camera, 21), s_Swerve));
      // aimtarget.whileTrue(s_Swerve.aimAtTarget());
      //driveToPos.whileTrue(new DriveToPosition(s_Swerve, () -> new Pose2d(0, 0, new Rotation2d(0)), new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0), new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1))));
      
      manualElevator.whileTrue(new RunCommand(elevator::openElevator, elevator).repeatedly());
      manualElevator.whileFalse(new RunCommand(elevator::stop, elevator));

      elevator_kapat.whileTrue(new RunCommand(elevator::closeElevator, elevator));
      elevator_kapat.whileFalse(new RunCommand(elevator::stop, elevator));

//      elevator_l4Button.whileTrue(new ElevatorCommand(elevator,  -892.0, -42,  -0.5));
      elevator_l4Button.whileFalse(new RunCommand(elevator::stop, elevator));
  
//      elevator_l3Button.whileTrue(new elevator_otonom(s_yukari, -456, -42,-0.4));
      elevator_l3Button.whileFalse(new RunCommand(elevator::stop, elevator));
  
//      elevator_l2Button.whileTrue(new elevator_otonom(s_yukari, -135, -37,-0.4));
      elevator_l2Button.whileFalse(new RunCommand(elevator::stop, elevator));
  
      climb_Bas.whileTrue(new RunCommand(climb::climbPress, climb));
      climb_Bas.whileFalse(new RunCommand(climb::climbStop, climb));
      
      climb_In.whileTrue(new RunCommand(climb::climbWithdraw, climb));
      climb_In.whileFalse(new RunCommand(climb::climbStop, climb));
  
      zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroGyro));
  
      xLock.whileTrue(Commands.runOnce((s_Swerve::lock), s_Swerve).repeatedly());
  
      shooter_Tukur.whileTrue(new RunCommand(intake::shooterSpit, intake));
      shooter_Tukur.whileFalse(new RunCommand(intake::shooterStop, intake));
  
      shooter_IcineAl.whileTrue(new RunCommand(intake::shooterIntake, intake));
      shooter_IcineAl.whileFalse(new RunCommand(intake::shooterStop, intake));
  
      Thread.sleep(10);

      SmartDashboard.putNumber("Elevator Encoder", elevator.getEncoderPosition());
       
       
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
      
    }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    s_Swerve.setMotorBrake(brake);
  }

  public void resetOdometry(Pose2d pose) {
    s_Swerve.resetOdometry(pose);
  }
  
  public void zeroGyro() {
    s_Swerve.zeroGyro();
  }

}
