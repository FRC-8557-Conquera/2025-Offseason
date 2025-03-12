package frc.robot.subsystems; 
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import frc.robot.NavX.AHRS;
import frc.robot.subsystems.Vision.Cameras;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import com.pathplanner.lib.config.RobotConfig;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {
  //public final AHRS gyro =  new AHRS(SPI.Port.kMXP, (byte)50);
  //private SwerveDriveOdometry swerveOdometry;
  private Field2d field;
  public SwerveDrive swerveDrive;
  private final boolean visionDriveTest = false;
  public Vision vision;
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  public Swerve() {
    RobotConfig config;
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      
      Thread.sleep(1000); // waiting for 1 second for the navx to complete the calibration before resetting the yaw
      config = RobotConfig.fromGUISettings();
      boolean enableFeedforward = true;
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.maxSpeed);
      config = RobotConfig.fromGUISettings();
      //RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
      AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speedsRobotRelative, moduleFeedForwards) -> {
          if (enableFeedforward)
          {
            swerveDrive.drive(
                speedsRobotRelative,
                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces()
                             );
          } else
          {
            swerveDrive.setChassisSpeeds(speedsRobotRelative);
          }
        },
        new PPHolonomicDriveController(
          new PIDConstants(5, 0, 0),
          new PIDConstants(4, 0, 0)
        ), 
        config,
        () -> {
          
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this
      );
      
      } 
    catch (Exception e) {
      Thread.currentThread().interrupt();
      e.printStackTrace();
      throw new RuntimeException("Failed to initialize RobotConfig", e);
    } 
    if(visionDriveTest) {
      setupPhotonVision();
      swerveDrive.stopOdometryThread();
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setAutoCenteringModules(false);
    swerveDrive.setHeadingCorrection(false);
      
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public Command aimAtTarget() {
    return run(() -> {
    System.out.println("Komut geldi");
      Optional<PhotonPipelineResult> resultO = Cameras.RAZER.getBestResult();
      if (resultO.isPresent())
      {
        System.out.println("cam");
        SmartDashboard.putString("vision", "Sonuc var");
        var result = resultO.get();
        if (result.hasTargets())
        {
        System.out.println("april geldi");
        SmartDashboard.putString("vision", "Hedef var");
          drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(result.getBestTarget()
                                                             .getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }

  public Command driveCommand( DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
     
        return run(() -> {
          
          //Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), speedRateSwerve);
        //driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
        
          swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
      
      });

    }
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveDrive.getModules()) {
    mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
   }

  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }
 
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red:false;
  }
  public void zeroGyroWithAlliance(){
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public void lock() {
    swerveDrive.lockPose();
  }
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  public void setMaximumSpeed(double speed) {
    swerveDrive.setMaximumAllowableSpeeds(speed, 9.424);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveDrive.getModules()) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
    
  }
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }
  public void driveFieldOriented(ChassisSpeeds velSpeeds){
    swerveDrive.driveFieldOriented(velSpeeds);
  }
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  
  }



  public double getTilt(){
    double pitch = swerveDrive.getPitch().getDegrees();
    double roll = swerveDrive.getRoll().getDegrees();
    if ((pitch+roll) >=0) {
      return Math.sqrt(pitch*pitch + roll*roll);
    } 
    else {
      return -Math.sqrt(pitch*pitch + roll*roll);
    }

}

/*public static double speedRateSwerve = 1.0; 


  public void incSpeed() {

    if(speedRateSwerve < 1.15 ) { 
      speedRateSwerve = speedRateSwerve + 0.1;
  } 

  }

  public void decSpeed() {
    
    if(speedRateSwerve > 0.15 ){
      speedRateSwerve = speedRateSwerve - 0.1;
    }

  }*/
  public double getPitch() {
    return swerveDrive.getPitch().getDegrees();
  }

  public double getRoll() {
    return swerveDrive.getRoll().getDegrees();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-swerveDrive.getYaw().getDegrees());
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule mod : swerveDrive.getModules()){

      mod.restoreInternalOffset();

    } 
  }
  
  public ChassisSpeeds getChassisSpeeds() {
    
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(
      swerveDrive.getModules()[0].getState(),
      swerveDrive.getModules()[1].getState(),
      swerveDrive.getModules()[2].getState(),
      swerveDrive.getModules()[3].getState());
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }

  @Override
  public void periodic() {
  if(visionDriveTest){
    swerveDrive.updateOdometry();
    vision.updatePoseEstimation(swerveDrive);
  }
    
    swerveDrive.updateOdometry();

    field.setRobotPose(swerveDrive.getPose());
/*  for (SwerveModule mod : swerveDrive.getModules()) {

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAbsolutePosition());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getRelativePosition());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putString(
          "Mod " + mod.moduleNumber + " Position", mod.getPosition().toString());
          
    }*/
    
  }

} 