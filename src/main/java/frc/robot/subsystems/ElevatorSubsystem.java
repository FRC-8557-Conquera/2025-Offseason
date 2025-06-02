package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(Constants.ElevatorConstants.leftMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(Constants.ElevatorConstants.rightMotorID, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final ProfiledPIDController controller = new ProfiledPIDController(
            Constants.ElevatorConstants.kP,
            Constants.ElevatorConstants.kI,
            Constants.ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxSpeed, Constants.ElevatorConstants.maxAccel)
    );
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            Constants.ElevatorConstants.kS,
            Constants.ElevatorConstants.kG, 
            Constants.ElevatorConstants.kV,
            Constants.ElevatorConstants.kA
    );
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40).openLoopRampRate(Constants.ElevatorConstants.rampRate);
        leftMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftMotor);
        rightMotor.configure(followerConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public double getPositionMeters() {
        return leftEncoder.getPosition() * Constants.ElevatorConstants.specialConstant;
    }

    public double getVelocityMetersPerSecond() {
        return (leftEncoder.getVelocity() / 60) * Constants.ElevatorConstants.specialConstant;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                elevatorFeedforward.calculateWithVelocities(getVelocityMetersPerSecond(), controller.getSetpoint().velocity)
                        + controller.calculate(getPositionMeters(), goal),
                -7,
                7);
        leftMotor.setVoltage(voltsOutput);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, 0.07);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

    public void stop() {
        leftMotor.set(0);
    }

    public void resetEncoder(){
        leftEncoder.setPosition(0);
    }

    public void openElevator() {
        leftMotor.set(0.5);
    }

    public void closeElevator() {
        leftMotor.set(-0.5);
    }
}
