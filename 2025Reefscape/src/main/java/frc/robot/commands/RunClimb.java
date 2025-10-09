
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class RunClimb extends Command {
  private Climb climb;
<<<<<<< HEAD
   private double targetPosition;
   private double speed;
=======
  private double speed;
>>>>>>> 5c6ecaf5df2cf5200d8e777b7eeb84d56bccc7dc

  /** Creates a new RunFastIntake.
   * @param - intake
   * @param - speed
   */

<<<<<<< HEAD
  public RunClimb(Climb climb, double targetPosition, double speed ) {
      this.climb = climb;
      this.targetPosition = targetPosition;
      this.speed = speed; 
=======
  public RunClimb(Climb climb, double speed) {
      this.climb = climb;
      this.speed = speed;
>>>>>>> 5c6ecaf5df2cf5200d8e777b7eeb84d56bccc7dc
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