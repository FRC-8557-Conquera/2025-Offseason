package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax shooter = new SparkMax(Constants.ShooterConstants.shooterID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax intakeLeft = new SparkMax(Constants.IntakeConstants.intakeLeftID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(Constants.IntakeConstants.intakeRightID, SparkLowLevel.MotorType.kBrushless);
    public void setShooter(double speed) {
        shooter.set(speed);
    }
}
