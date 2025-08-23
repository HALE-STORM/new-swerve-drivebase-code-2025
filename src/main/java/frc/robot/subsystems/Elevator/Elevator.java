package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;

public class Elevator extends SubsystemBase {
    final TalonFX m_talonFX = new TalonFX(4); // m_talonFX defined as Elevator Motor (ID:4)
    public Elevator() {
        var slot0Configs = new Slot0Configs(); // create new Slot0Configs() objet called slot0Configs
        slot0Configs.kP = 0.8; // Proportional Voltage Adjustment to current error
        // E.G. at kP=0.8, 1 Rotation Error = 0.8V, 2 = 1.6 V, etc.
        slot0Configs.kI = 0.004; // Sum Voltage Adjustment to creep over error
        // E.G. 2 Rotation Error, kP can't fix, Voltage=kI*SUM(Error*Elaspsed Time) *Sum in respect to time*
        slot0Configs.kD = 0.01; // Adjusts Voltage based on Derivative (Gradient) of movement. Prevents over/under shoot.
        // E.G. Aim for 10 Rotations at rate of 1.2rps, as r->10, rps descreases to ensure it reaches 10 with min RPS
        slot0Configs.kG = 0.7; // Voltage needed to overcome gravity.
        // E.G. To get value Trial and Error, calcs possible but time consuming.
        m_talonFX.getConfigurator().apply(slot0Configs);
    }
    public final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    public Command goToHeight(double heightRotations) {
        // create a position closed-loop request, voltage output, slot 0 configs
        // set position to 10 rotations
        return run(() -> m_talonFX.setControl(m_request.withPosition(heightRotations)));
    }






}
