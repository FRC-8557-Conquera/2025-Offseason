package frc.robot.subsystems;

import java.lang.module.ResolutionException;
import java.util.logging.LogManager;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter_al;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Peripheral extends SubsystemBase {
    public final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(Constants.yukari.kS, Constants.yukari.kG, Constants.yukari.kV, Constants.yukari.kA);
    public final SparkMax elevatorsolm = new SparkMax(Constants.yukari.solelevator, MotorType.kBrushless);
    public final SparkMax elevatatorsagm = new SparkMax(Constants.yukari.sagelevator, MotorType.kBrushless);
    public final SparkMax climbmotor = new SparkMax(Constants.yukari.climbmotor, MotorType.kBrushless);
    public final RelativeEncoder enccodersol = elevatorsolm.getEncoder();
    public final RelativeEncoder enccodersag = elevatatorsagm.getEncoder();
    public final SparkMax shooter = new SparkMax(Constants.yukari.shooter, MotorType.kBrushless);
    public final SparkMax shooteraci = new SparkMax(Constants.yukari.shootertaci, MotorType.kBrushless);
    public final RelativeEncoder shooterencoder = shooteraci.getEncoder();
    public final ProfiledPIDController elevatorController = new ProfiledPIDController(
            Constants.yukari.kP, Constants.yukari.kI, Constants.yukari.kD,
            new Constraints(1.0, 0.5)); // Maks hız ve ivme (örnek değerler)
    public Peripheral(){
        shooterencoder.setPosition(0);
        enccodersol.setPosition(0);
        enccodersag.setPosition(0);
        elevatorController.setGoal(0);
        elevatorsolm.setInverted(false); //elevator - yön
        elevatatorsagm.setInverted(true);//elevator + yön
        climbmotor.setInverted(false);
    }
    
    @Override
    public void periodic() {
        //Display sensor readings to ShuffleBoard
        SmartDashboard.putNumber("Elevator Velocity1", enccodersol.getVelocity());
        SmartDashboard.putNumber("Elevator Velocity2", enccodersag.getVelocity());
        SmartDashboard.putNumber("Elevator Encoder2", enccodersag.getPosition());
        SmartDashboard.putNumber("Elevator Encoder1", enccodersol.getPosition());
        SmartDashboard.putNumber("shooter encoder", shooterencoder.getPosition());
    }
    
    public void elevatorAc(){
        elevatorsolm.set(-0.5);
        elevatatorsagm.set(-0.5);
    }
    public void elevatorKapat(){
        elevatorsolm.set(0.1);
        elevatatorsagm.set(0.1);
        
    }
    public void elevatorDurdur(){
        elevatorsolm.set(-0.05);
        elevatatorsagm.set(-0.05);
    }
    
    public void setShooterAngle(double targetAngle) {
        // shooteraci'nin kapalı döngü kontrolü modunda olduğunu ve konum kontrolü yapılandırıldığını varsayıyoruz.
        shooteraci.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
    }


    public void climbBas(){
        climbmotor.set(0.9);
    }
    public void climbIn(){
        climbmotor.set(-0.9);
    }
    public void climbDur(){
        climbmotor.set(0);
    }
    public void shooterTukur(){
        shooter.set(-0.5);
    }
    public void shooteryavas(){ 
        shooter.set(0.1);
    }
    public void shooterIcineal(){
        shooter.set(0.8);
    }
    public void shooterDurdur(){
        shooter.set(0);
    }
    public void ShooteraciDurdur(){
        shooteraci.set(0);
    }
    public void ShooteraciYukari(){
        shooteraci.set(0.4);
    }
    public void ShooteraciAsagi(){
        shooteraci.set(-0.3);
    }
    public double getEncoderPosition() {
        return enccodersol.getPosition(); 
    }
    public double getEncoderVelocity() {
        return enccodersol.getVelocity();
    }
    public double getEncoderAciPosition(){
        return shooterencoder.getPosition();
    }
    public void setElevatorOutput(double output) {
        elevatorsolm.set(-output);
        elevatatorsagm.set(-output);
    }
    public void resetEncoder(){
        enccodersol.setPosition(0);
    }


}