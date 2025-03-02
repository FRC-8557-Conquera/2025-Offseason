package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class elevator_to_autonom_kapat extends Command {
    private final Peripheral elevator;
    private final double target;      // Örneğin -1 (encoder biriminde, hedef konum)
    private final double motorSpeed;  // Motorun sabit çıkış hızı, örneğin 0.4

    public elevator_to_autonom_kapat(Peripheral elevator, double target, double motorSpeed) {
        this.elevator = elevator;
        this.target = target;
        this.motorSpeed = motorSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // İsteğe bağlı: encoder sıfırlaması yapılabilir
        elevator.elevatorDurdur();

    }

    @Override
    public void execute() {
        double currentPosition = elevator.getEncoderPosition();
        
        if (currentPosition < target) { 
            // Hedefe ulaşılmamışsa sabit hızda çalıştır (hedefe ulaşana kadar)
            elevator.setElevatorOutput(-motorSpeed);
        } else {
            // Hedefe ulaşıldığında motorları durdur
            elevator.elevatorDurdur();
        }
    }

    @Override
    public boolean isFinished() {
        // Encoder değeri hedefe ulaştığında (veya daha düşük olduğunda) komut sona ersin.
        return elevator.getEncoderPosition() >= target;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorDurdur();
    }
}
