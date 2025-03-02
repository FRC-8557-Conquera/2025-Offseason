package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class elevator_otonom extends Command {
    private final Peripheral elevator;
    private final double elevatorTarget;  // Elevator hedef encoder değeri (örneğin -32)
    private final double shooterTarget;   // Shooter açı hedef encoder değeri (örneğin istenen açı değeri)
    private final double elevatorSpeed;   // Elevator motor hızı (örneğin 0.6)
    // Shooter için doğrudan hız parametresi yerine, shooter motorunun sabit çıkışlarını kullanacağız.
    private final double shooterTolerance = 0.3; // Shooter açısı için tolerans (encoder biriminde)

    public elevator_otonom(Peripheral elevator, double elevatorTarget, double shooterTarget, double elevatorSpeed) {
        this.elevator = elevator;
        this.elevatorTarget = elevatorTarget;
        this.shooterTarget = shooterTarget;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // İsteğe bağlı: Komut başlangıcında encoder sıfırlaması yapılabilir.
        // elevator.resetEncoder(); 
        elevator.elevatorDurdur();
    }

    @Override
    public void execute() {
        // Elevator kontrolü:
        double currentElevatorPos = elevator.getEncoderPosition();
        // Encoder değerleri 0 ile -35 arasında çalışıyorsa, hedefe ulaşmamışsa (örn. -32'ye ulaşmamışsa) hareket ettir.
        if (currentElevatorPos > elevatorTarget) {
            // Elevator yukarı çıkarken motor çıkışı negatif (örn. -elevatorSpeed)
            elevator.setElevatorOutput(-elevatorSpeed); // Peripheral içindeki setElevatorOutput() metodu zaten çıktı tersliyse uygun şekilde ayarlanmış olabilir.
        } else {
            elevator.elevatorDurdur();
        }
        
        // Shooter açı kontrolü:
        double currentShooterPos = elevator.getEncoderAciPosition();
        if (currentShooterPos < shooterTarget - shooterTolerance) {
            // Hedefin altındaysa shooter açısını artır (örn. shooter motoru pozitif çıkış veriyor)
            elevator.ShooteraciYukari();
        } else if (currentShooterPos > shooterTarget + shooterTolerance) {
            // Hedefin üstündeyse shooter açısını azalt
            elevator.ShooteraciAsagi();
        } else {
            // Hedefe yakınsa shooter motorunu durdur
            elevator.ShooteraciDurdur();
        }
    }

    @Override
    public boolean isFinished() {
        // Komut, elevator hedefe ulaşmış ve shooter açısı hedefe (tolerans dahilinde) ulaşmışsa biter.
        double currentElevatorPos = elevator.getEncoderPosition();
        double currentShooterPos = elevator.getEncoderAciPosition();
        return (currentElevatorPos <= elevatorTarget) && (Math.abs(currentShooterPos - shooterTarget) < shooterTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorDurdur();
        elevator.ShooteraciDurdur();
    }

 {
    
 }
}
