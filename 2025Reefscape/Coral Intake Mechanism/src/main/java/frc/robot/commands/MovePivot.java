package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class MovePivot extends Command {
    private final Pivot m_pivot; // The pivot subsystem
    private final double m_targetPosition; // Target position for the pivot

    /**
     * Creates a new RunPivot command.
     *
     * @param pivot The pivot subsystem this command will run on
     * @param targetPosition The position to move the pivot to
     */
    public MovePivot(Pivot pivot, double targetPosition) {
        m_pivot = pivot; // Assign the provided pivot subsystem
        m_targetPosition = targetPosition; // Set the target position
        addRequirements(m_pivot); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        m_pivot.moveTo(m_targetPosition); // Set the goal for the pivot
    }

    @Override
    public void execute() {
        // The periodic method in Pivot handles execution
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isReached(); // Check if the pivot has reached its goal
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.stop(); // Stop the pivot when the command ends
    }
   
    }
