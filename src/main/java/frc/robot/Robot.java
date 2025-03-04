// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
 /* private AddressableLED led1;
  private AddressableLEDBuffer ledBuffer1;
  private AddressableLED led2;
  private AddressableLEDBuffer ledBuffer2;*/
  
  @Override
  public void robotInit() {
    // PWM Port 1'deki LED şeridi başlat (82 LED)
   /* led1 = new AddressableLED(0);
    ledBuffer1 = new AddressableLEDBuffer(83);
    led1.setLength(ledBuffer1.getLength());
    
    // Tüm LED'leri mavi renge ayarla (RGB: 0, 0, 255)
    for (int i = 0; i < ledBuffer1.getLength(); i++) {
        ledBuffer1.setRGB(i, 0, 0, 0);
    }
    
    // Buffer'ı LED şeridine gönder ve şeridi başlat
    led1.setData(ledBuffer1);
    led1.start();

    // PWM Port 2'deki LED şeridi başlat (83 LED)
    led2 = new AddressableLED(1);
    ledBuffer2 = new AddressableLEDBuffer(82);
    led2.setLength(ledBuffer2.getLength());

    // Tüm LED'leri mavi renge ayarla (aynı renk, RGB: 0, 0, 255)
    for (int i = 0; i < ledBuffer2.getLength(); i++) {
        ledBuffer2.setRGB(i, 0, 0, 0);
    }
    
    led2.setData(ledBuffer2);
    led2.start();* */
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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
