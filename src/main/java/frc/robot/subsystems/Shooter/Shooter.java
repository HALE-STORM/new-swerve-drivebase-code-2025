package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter  extends SubsystemBase {
    private final TalonFX ShooterMotor = new TalonFX(10);
    public final DigitalInput beamBreak = new DigitalInput(0);

public Shooter(){
    setDefaultCommand(stopShooter().ignoringDisable(true));
}

   
    public Command runShooter() {
      return Commands.run(
          () -> ShooterMotor.setVoltage(-4),
  
          this
      );
    }

    public Command autoIntake() {
      return Commands.run(
        () -> ShooterMotor.setVoltage(-2.25),

        this
        );
    }
  
  
    public Command ejectShooter() {
      return Commands.run(
          () -> ShooterMotor.setVoltage(0.5),
  
          this
      );
    }
  
    public Command stopShooter(){
      return Commands.run(
        () -> ShooterMotor.setVoltage(0),
        this
      );
    }

  
    
    public Command smartShooter(){
      return autoIntake().until(beamBroken)
      //.andThen(runEjectShooter().until(beamNotBroken))
      .andThen(stopShooter());
    }




  public BooleanSupplier beamBroken = () -> !beamBreak.get();
  public BooleanSupplier beamNotBroken = () -> beamBreak.get();


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}




