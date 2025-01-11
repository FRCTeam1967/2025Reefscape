package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntake;
import frc.robot.commands.MovePivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot; // Import the Pivot subsystem

public class RobotContainer {
    // create instances of the subsystems
    private final Intake m_intake = new Intake();
    private final Pivot m_pivotL1 = new Pivot.L1(); // Example for L1 pivot
    private final Pivot m_pivotL2 = new Pivot.L2(); // Example for L2 pivot
    private final XboxController m_controller = new XboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // bind left joystick to run intake
        new Trigger(() -> Math.abs(m_controller.getLeftY()) > 0.1)
            .whileTrue(new RunIntake(m_intake, () -> m_controller.getLeftY()));

        // Bind buttons for pivot control
        new Trigger(() -> m_controller.getAButtonPressed())
            .onTrue(new MovePivot(m_pivotL1, 1.0)); // Move L1 pivot to position 1.0

        new Trigger(() -> m_controller.getBButtonPressed())
            .onTrue(new MovePivot(m_pivotL2, 1.0)); // Move L2 pivot to position 1.0

        new Trigger(() -> m_controller.getXButtonPressed())
            .onTrue(new MovePivot(m_pivotL1, 0.5)); // Move L1 pivot to position 0.5

        new Trigger(() -> m_controller.getYButtonPressed())
            .onTrue(new MovePivot(m_pivotL2, 0.5)); // Move L2 pivot to position 0.5
    }
}