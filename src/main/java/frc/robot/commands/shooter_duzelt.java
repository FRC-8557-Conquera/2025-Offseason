package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;
import com.revrobotics.spark.SparkBase.ControlType;

public class shooter_duzelt extends Command {
    private final Peripheral shooterSubsystem;
    private final double targetAngle = 9.07; // Hedef shooter açısı (derece cinsinden)
    private final double tolerance = 0.1;     // Tolerans değeri

    public shooter_duzelt(Peripheral shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // İsteğe bağlı: shooter encoder sıfırlanabilir
    }

    @Override
    public void execute() {
        // Shooter açısını kapalı döngü ile hedef açıya getir
        shooterSubsystem.setShooterAngle(targetAngle);
        shooterSubsystem.shooteryavas();
    }

    @Override
    public boolean isFinished() {
        // Mevcut shooter açısı hedefe yakınsa komut biter
        return Math.abs(shooterSubsystem.getEncoderAciPosition() - targetAngle) < tolerance;
        
    }

    @Override
    public void end(boolean interrupted) {
        // Komut biterken shooter motorunu durdur
        shooterSubsystem.ShooteraciDurdur();
        shooterSubsystem.shooteryavas();
    }
}
