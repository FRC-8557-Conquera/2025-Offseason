package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class elevator_to_autonom extends Command {
    private final Peripheral elevator;
    private final double target;      // Örneğin -32 (encoder biriminde)
    private final double motorSpeed;  // Maksimum motor hızı, örneğin 0.4 (pozitif değer, çıkış –0.4)

    public elevator_to_autonom(Peripheral elevator, double target, double motorSpeed) {
        this.elevator = elevator;
        this.target = target;
        this.motorSpeed = motorSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // İsteğe bağlı: Encoder sıfırlama veya hedef ayarlaması yapılabilir.
    }

    @Override
    public void execute() {
        double currentPosition = elevator.getEncoderPosition();
        // Eğer elevator henüz hedefe ulaşmamışsa (daha az negatifse), sabit hızda çalıştır.
        if (currentPosition > target) {
            elevator.setElevatorOutput(-motorSpeed);
        } else {
            // Hedefe ulaşıldığında motorları durdur.
            elevator.elevatorDurdur();
        }
    }

    @Override
    public boolean isFinished() {
        // Encoder değeri hedefe ulaştığında komut sona ersin.
        return elevator.getEncoderPosition() <= target;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorDurdur();
    }
}
