package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class RunClimb extends Command {
  private final Climb climb;
  private final double speed;

  /** Creates a new RunFastIntake.
   * @param - intake
   * @param - speed
   */

  public RunClimb(Climb climb, double speed) {
    this.climb = climb;
    this.speed = speed;

    addRequirements(climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      climb.runClimb(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climb.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}