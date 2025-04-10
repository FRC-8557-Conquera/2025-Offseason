package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MU;
import frc.robot.subsystems.Peripheral;

public class HybridCommand extends Command {
    private final Peripheral peripheral;
    private final double elevatorTarget;  // Elevator hedef encoder değeri (örneğin -32)
    private final double shooterTarget;   // Shooter açı hedef encoder değeri (örneğin istenen açı değeri)
    private final double elevatorSpeed;   // Elevator motor hızı (örneğin 0.6)
    // Shooter için doğrudan hız parametresi yerine, shooter motorunun sabit çıkışlarını kullanacağız.
    private final double shooterTolerance = 0.3; // Shooter açısı için tolerans (encoder biriminde)

    public HybridCommand(Peripheral elevator, double elevatorTarget, double shooterTarget, double elevatorSpeed) {
        this.peripheral = elevator;
        this.elevatorTarget = elevatorTarget;
        this.shooterTarget = shooterTarget;
        this.elevatorSpeed = elevatorSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // İsteğe bağlı: Komut başlangıcında encoder sıfırlaması yapılabilir.
        // elevator.resetEncoder();
        peripheral.elevatorDurdur();
    }

    @Override
    public void execute() {
        // Elevator kontrolü:
        double currentElevatorPos = peripheral.getEncoderPosition();
        if(!MU.isNear(currentElevatorPos, elevatorTarget, 0.3)) {
            if(currentElevatorPos > elevatorTarget) {
                peripheral.elevatorKapat();
            } else {
                peripheral.elevatorAc();
            }
        }

        // Shooter açı kontrolü:
        double currentShooterPos = peripheral.getEncoderAciPosition();
        if(!MU.isNear(currentShooterPos, shooterTarget, 0.3)) {
            if(currentShooterPos < shooterTarget) {
                peripheral.ShooteraciYukari();
            } else {
                peripheral.ShooteraciAsagi();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Komut, elevator hedefe ulaşmış ve shooter açısı hedefe (tolerans dahilinde) ulaşmışsa biter.
        double currentElevatorPos = peripheral.getEncoderPosition();
        double currentShooterPos = peripheral.getEncoderAciPosition();
        return (MU.isNear(currentElevatorPos, elevatorTarget, 0.3)) && (MU.isNear(currentShooterPos, shooterTarget, 0.3));
    }

    @Override
    public void end(boolean interrupted) {
        peripheral.elevatorDurdur();
        peripheral.ShooteraciDurdur();
    }
}
