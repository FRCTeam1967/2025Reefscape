
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private final Intake intake;
  private final double speed;

  /** Creates a new RunFastIntake.
   * @param - intake
   * @param - speed
   */

  public RunIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      intake.runIntake(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}