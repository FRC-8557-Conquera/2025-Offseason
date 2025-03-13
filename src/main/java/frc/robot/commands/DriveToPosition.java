package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Cameras;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonTargetSortMode;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

public class DriveToPosition extends Command {

    @FunctionalInterface
    public interface ErrorConsumer {
        void apply(double xError, double yError, double thetaError);
    }
    private final Swerve m_swerve;
    private final HolonomicDriveController m_driveController;
    private SimpleMotorFeedforward m_translationXFeedforward;
    private SimpleMotorFeedforward m_translationYFeedforward;
    private SimpleMotorFeedforward m_rotationFeedforward;
    private final PIDController m_thetaController = new PIDController(0, 0, 0);
    private Supplier<Pose2d> m_targetPoseSupplier;
    private final List<ErrorConsumer> m_errorConsumers = new ArrayList<>();

    public DriveToPosition(
        Swerve swerve,
        Supplier<Pose2d> targetPose,
        PIDController pidX,
        PIDController pidY,
        ProfiledPIDController pidTheta) {
      m_swerve = swerve;
      m_driveController = new HolonomicDriveController(pidX, pidY, pidTheta);
      m_translationXFeedforward = new SimpleMotorFeedforward(0, 0, 0);
      m_translationYFeedforward = new SimpleMotorFeedforward(0, 0, 0);
      m_rotationFeedforward = new SimpleMotorFeedforward(0, 0, 0);
      m_targetPoseSupplier = targetPose;
      addRequirements(swerve);
    }

    public DriveToPosition(Swerve swerve, Supplier<Pose2d> targetPose) {
      this(
          swerve,
          targetPose,
          new PIDController(0, 0, 0),
          new PIDController(0, 0, 0),
          new ProfiledPIDController(0, 0, 0, null));
    }

    public DriveToPosition(Swerve swerve) {
      this(swerve, null);
    }

    public DriveToPosition withPositionKP(double kP) {
      m_driveController.getXController().setP(kP);
      m_driveController.getYController().setP(kP);
      return this;
    }

    public DriveToPosition withPositionKI(double kI) {
      m_driveController.getXController().setI(kI);
      m_driveController.getYController().setI(kI);
      return this;
    }

    public DriveToPosition withPositionKD(double kD) {
      m_driveController.getXController().setD(kD);
      m_driveController.getYController().setD(kD);
      return this;
    }

    public DriveToPosition withAngleKP(double kP) {
      m_thetaController.setP(kP);
      m_driveController.getThetaController().setP(kP);
      return this;
    }

    public DriveToPosition withAngleKI(double kI) {
      m_thetaController.setI(kI);
      m_driveController.getThetaController().setI(kI);
      return this;
    }

    public DriveToPosition withAngleKD(double kD) {
      m_thetaController.setD(kD);
      m_driveController.getThetaController().setD(kD);
      return this;
    }

    public DriveToPosition withAnglePIDConstraints(TrapezoidProfile.Constraints constraints) {
      // m_thetaController.setConstraints(constraints);
      m_driveController.getThetaController().setConstraints(constraints);
      return this;
    }

    public DriveToPosition withTargetPosition(Supplier<Pose2d> targetSupplier) {
      this.m_targetPoseSupplier = targetSupplier;
      return this;
    }

    public DriveToPosition withErrorAction(ErrorConsumer consumer) {
      this.m_errorConsumers.add(consumer);
      return this;
    }

    public DriveToPosition withPositionFeedforward(double kS, double kV, double kA) {
      this.m_translationXFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
      this.m_translationYFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
      return this;
    }

    public DriveToPosition withRotationFeedforward(double kS, double kV, double kA) {
      this.m_rotationFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
      return this;
    }

    @Override
    public void execute() {
      Pose2d currentPose = m_swerve.getPose();

      ChassisSpeeds speeds = calculateSpeeds(currentPose, m_targetPoseSupplier.get());

      if (!DriverStation.isFMSAttached()) {
        SmartDashboard.putNumber("Drive/xErr", m_driveController.getXController().getError());
        SmartDashboard.putNumber("Drive/yErr", m_driveController.getYController().getError());
        SmartDashboard.putNumber(
            "Drive/HeadingErr", m_driveController.getThetaController().getPositionError());
      }

      for (ErrorConsumer errorConsumer : m_errorConsumers) {
        errorConsumer.apply(
            m_driveController.getXController().getError(),
            m_driveController.getYController().getError(),
            m_driveController.getThetaController().getPositionError());
      }

      m_swerve.drive(speeds);
    }

    public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d target) {
      var speeds =
          m_driveController.calculate(
              currentPose, target, 0.0, target.getRotation().rotateBy(Rotation2d.k180deg));
      double vx =
          speeds.vxMetersPerSecond + m_translationXFeedforward.calculate(speeds.vxMetersPerSecond);
      double vy =
          speeds.vyMetersPerSecond + m_translationYFeedforward.calculate(speeds.vyMetersPerSecond);

      double error =
          Math.IEEEremainder(
              target.getRotation().getMeasure().in(Degrees)
                  - currentPose.getRotation().getDegrees(),
              360);
      double omega = -m_thetaController.calculate(0, error);

      omega += m_rotationFeedforward.calculate(omega);

      if (Math.abs(vx) < 0.1) {
        vx = 0;
      }
      if (Math.abs(vy) < 0.1) {
        vy = 0;
      }
      if (Math.abs(omega) < 0.1) {
        omega = 0;
      }

      return new ChassisSpeeds(vx, vy, omega);
    }

    @Override
    public boolean isFinished() {
      Pose2d targetPose = m_targetPoseSupplier.get();
      double poseErr =
          Math.abs(m_swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()));
      boolean positionReached =
          poseErr < Constants.Swerve.kTranslationVarianceThreshold;
      boolean angleReached =
          Math.abs(m_swerve.getHeading().getDegrees() - targetPose.getRotation().getDegrees())
              < Constants.Swerve.kAngleVarianceThreshold;

      return (positionReached && angleReached);
    }

    @Override
    public void end(boolean interrupted) {
      m_swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
  }
