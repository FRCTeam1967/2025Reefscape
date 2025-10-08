
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class RunClimb extends Command {
  private Climb climb;
   private double targetPosition;
   private double speed;

  /** Creates a new RunFastIntake.
   * @param - intake
   * @param - speed
   */

  public RunClimb(Climb climb, double targetPosition, double speed ) {
      this.climb = climb;
      this.targetPosition = targetPosition;
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