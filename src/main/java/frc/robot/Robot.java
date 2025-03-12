// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;
import frc.robot.subsystems.Peripheral;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Cameras;
import swervelib.SwerveDrive;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final Joystick driver = new Joystick(0); 


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    PortForwarder.add(5800, "photonvision.local", 5800);
    
  }
  @Override
  public void robotPeriodic() {
    // Update the buffer with the rainbow animation

    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.zeroGyro();
    m_robotContainer.resetOdometry(new Pose2d(0,0,Rotation2d.kZero));
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
   if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
                // Calculate drivetrain commands from Joystick values
    double forward = -driver.getAxisType(1) * Constants.MAX_SPEED;
    double strafe = -driver.getAxisType(0) * Constants.MAX_SPEED;
    double turn = -driver.getAxisType(2) * Constants.Swerve.maxAngularVelocity;

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = Cameras.RAZER.getLatestResult();
    if (results.isPresent()) {
        // Camera processed a new frame since last
        // Get the result from the Optional.
        var result = results.get();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        }
    }

    // Auto-align when requested
    if (driver.getRawButtonPressed(5) && targetVisible) {
        // Driver wants auto-alignment to tag 7
        // And, tag 7 is in sight, so we can turn toward it.
        // Override the driver's turn command with an automatic one that turns toward the tag.
        turn = -1.0 * targetYaw * 0.1 * Constants.Swerve.maxAngularVelocity;
    }

    // Command drivetrain motors based on target speeds
    m_robotContainer.s_Swerve.driveFieldOriented(new ChassisSpeeds(forward, strafe, turn));

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
