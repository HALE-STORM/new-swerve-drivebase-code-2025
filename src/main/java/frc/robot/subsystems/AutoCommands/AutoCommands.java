package frc.robot.subsystems.AutoCommands;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoCommands extends SubsystemBase {
    public Shooter shooter = new Shooter();
    public Elevator botelevator = new Elevator();

    public double firstLevel = 13.036621; //13.036621 / 13.331055
    public double algaeFirst = 9.013672; //9.013672
    public double secondLevel = 20.615723; //20.615723
    public double algaeSecond = 16.10644; //16.10644
    public double elevatorBase = 0.5;

    
    public Command elevatorHeightFirst() {
        return botelevator.goToHeight(firstLevel);
    }

    public Command elevatorHeightSecond() {
        return botelevator.goToHeight(secondLevel);
    }

    public Command elevatorFirstAlgae() {
        return botelevator.goToHeight(algaeFirst);
    }

    public Command elevatorSecondAlgae() {
        return botelevator.goToHeight(algaeSecond);
    }

    public Command smartShooter() {
        return shooter.smartShooter();
    }

    public Command smartIntake() {
        return shooter.smartIntake();
    }

    public Command autoIntake() {
        return shooter.autoIntake();
    }

    public Command resetElevator() {
        return botelevator.goToHeight(elevatorBase);
    }

    public Command runShooter() {
        return shooter.runShooter();
    }

    public Command ejectShooter() {
        return shooter.ejectShooter();
    }

    public Command stopShooter() {
        return shooter.stopShooter();
    }
}
