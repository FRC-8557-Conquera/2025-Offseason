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

import javax.swing.colorchooser.DefaultColorSelectionModel;

import org.photonvision.PhotonCamera;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.Vision.Cameras;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import frc.robot.Robot;
import frc.robot.Constants.yukari;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
public class RobotContainer {
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
 
  private final JoystickButton aimattarget = 
  new JoystickButton(driver, 5);

  // private final JoystickButton photon =
  //new JoystickButton(driver, 9);

  //private final JoystickButton driveToPos =
  // new JoystickButton(driver, 10);


  public final Swerve s_Swerve = new Swerve();
  public final Peripheral s_yukari = new Peripheral();
  // 
  //public final Vision vision = new Vision(s_Swerve::getPose, new edu.wpi.first.wpilibj.smartdashboard.Field2d());

  public final Vision s_Vision = s_Swerve.vision;

  public final climb_in climb_in = new climb_in(s_yukari);
  public final climb_bas climb_bas = new climb_bas(s_yukari); 

  //public final shooter_vur shooter_tukur = new shooter_vur(s_yukari);
  //public final shooter_al shooter_icineal = new shooter_al(s_yukari);

  public final shooter_yukari shooter_aci_yukari = new shooter_yukari(s_yukari);
  public final shooter_asagi shooter_aci_asagi = new shooter_asagi(s_yukari);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(s_Swerve.getSwerveDrive(), () -> -driver.getY(), () -> -driver.getX())
  .withControllerRotationAxis(() -> -driver.getZ())
  .deadband(Constants.Swerve.stickDeadband)
  .scaleTranslation(0.8) // YAVASLATMA!!!!
  .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3)).headingWhile(true);

  Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
  Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);
  // A chooser for autonomous commands

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    PhotonCamera camera = s_Vision.getCamera();
    NamedCommands.registerCommand("Shooter_al", shooter_saniye_al(s_yukari));
    NamedCommands.registerCommand("shooter_tukur", shooter_saniye_tukur(s_yukari));
    NamedCommands.registerCommand("elevator_l4", elevator_otonom(s_yukari, -830., -30, -0.6)); 
    NamedCommands.registerCommand("elevator_l3", elevator_otonom(s_yukari, -16.5, -28, -0.6));
    NamedCommands.registerCommand("elevator_l2", elevator_otonom(s_yukari, -8, -28,-0.6));
    NamedCommands.registerCommand("erene_duzelt", elevator_otonom(s_yukari, -1, -12.0, 0.3));
    NamedCommands.registerCommand("elevator_kapa", elevator_kapat(s_yukari, -2, 0.3));
    NamedCommands.registerCommand("Shooter_duzelt", elevator_otonom(s_yukari, -0, -20.23,-0.3));
    NamedCommands.registerCommand("alignApril", alignWithAprilTag(s_Swerve, camera, 13));
    //NamedCommands.registerCommand("drivetopose", drivetoposition(s_Swerve, () -> new Pose2d(0.140, 1.335, new Rotation2d(180)), new PIDController(5, 0, 0), new PIDController(4, 0, 0), new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1))));


    DriverStation.silenceJoystickConnectionWarning(true);
    s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
    configureButtonBindings();
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putString("pose", s_Swerve.getPose().toString());
    s_Swerve.resetModulesToAbsolute();
  
  
  }
  private Command shooter_saniye_tukur(Peripheral s_yukari){
    return new shooter_saniye_tukur(s_yukari, 1.5);
  }
  private Command shooter_saniye_al(Peripheral s_yukari){
    return new shooter_saniye_al(s_yukari,6);
  }
 private Command drivetoposition(Swerve s_Swerve, Supplier<Pose2d> pose, PIDController pidX, PIDController pidY, ProfiledPIDController pidTheta){
    return new DriveToPosition(s_Swerve, () -> new Pose2d(0, 0, new Rotation2d(0)), new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0), new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1)));
 }
  

  private Command alignWithAprilTag(Swerve s_Swerve, PhotonCamera camera, int tagID) {
    return new AlignWithAprilTag(s_Swerve, camera, tagID);
  }

  private Command elevator_otonom(Peripheral s_yukari,double position, double shooterTarget, double speed){
    return new elevator_otonom(s_yukari,position,shooterTarget,speed);
  }
  private Command elevator_kapat(Peripheral s_yukari, double position, double speed) {
    return new elevator_to_autonom_kapat(s_yukari, position, speed);
  }
  //  private Command decSpeed (Swerve s_Swerve, double maxSpeed){
  //    return new RunCommand(() -> s_Swerve, s_Swerve.setMaximumSpeed -= 1);
  //  }

  private void configureButtonBindings() {

    try {
    PhotonCamera camera = s_Vision.getCamera();  // Vision alt sisteminizin getCamera() metodunu ekleyin.
    aimattarget.whileTrue(new AlignWithAprilTag(s_Swerve, camera,21));
    
    //photon.whileTrue(new photonvision(s_Swerve, 7));
    
    shooter_duzelt.whileTrue(new elevator_otonom(s_yukari, -1, -8, 0.3));
    shooter_duzelt.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));

    // aimtarget.whileTrue(s_Swerve.aimAtTarget());

    //driveToPos.whileTrue(new DriveToPosition(s_Swerve, () -> new Pose2d(0, 0, new Rotation2d(0)), new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0), new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1))));
    
    elevator_l4Button.whileTrue(new elevator_otonom(s_yukari,  -830.0, -30,  -0.4));
    elevator_l4Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));

    elevator_l3Button.whileTrue(new elevator_otonom(s_yukari, -16.5, -30,-0.4));
    elevator_l3Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));

    elevator_l2Button.whileTrue(new elevator_otonom(s_yukari, -8, -28.5,-0.4));
    elevator_l2Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));

    shooter_Aci_Asagi.whileTrue(new RunCommand(() -> s_yukari.ShooteraciAsagi(), s_yukari));
    shooter_Aci_Asagi.whileFalse(new RunCommand(() -> s_yukari.ShooteraciDurdur(), s_yukari));

    shooter_Aci_Yukari.whileTrue(new RunCommand(()-> s_yukari.ShooteraciYukari(), s_yukari));
    shooter_Aci_Yukari.whileFalse(new RunCommand(()-> s_yukari.ShooteraciDurdur(), s_yukari));

    elevator_kapat.whileTrue(new elevator_to_autonom_kapat(s_yukari, -10.0, 0.2));
    elevator_kapat.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));


    climb_Bas.whileTrue(new RunCommand(() -> s_yukari.climbBas(), s_yukari));
    climb_Bas.whileFalse(new RunCommand(() -> s_yukari.climbDur(), s_yukari));
    
    climb_In.whileTrue(new RunCommand(() -> s_yukari.climbIn(), s_yukari));
    climb_In.whileFalse(new RunCommand(() -> s_yukari.climbDur(), s_yukari));

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    xLock.whileTrue(Commands.runOnce((() -> s_Swerve.lock()), s_Swerve).repeatedly());

    shooter_Tukur.whileTrue(new RunCommand(() -> s_yukari.shooterTukur(), s_yukari));
    shooter_Tukur.whileFalse(new RunCommand(() -> s_yukari.shooterDurdur(), s_yukari));

    shooter_IcineAl.whileTrue(new RunCommand(() -> s_yukari.shooterIcineal(), s_yukari));
    shooter_IcineAl.whileFalse(new RunCommand(() -> s_yukari.shooterDurdur(), s_yukari));

    Thread.sleep(10);
     
     
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
