package first.robot.commands;

import org.wpilib.command3.Command;

import first.robot.subsystems.Drivetrain;

public class Autos {

    public static Command driveTurnDrive(Drivetrain drivetrain) {
        return Command.sequence(
            drivetrain.driveDistance(1.0),
            drivetrain.turnToHeading(90),
            drivetrain.driveDistance(1.0)
        )
        .named ("Drive-Turn-Drive");
    }

    public static Command driveTurnDriveCoroutine(Drivetrain drivetrain){
        return Command.noRequirements(coroutine -> {
            coroutine.await(drivetrain.driveDistance(1.0));
            coroutine.await(drivetrain.turnToHeading(90));
            coroutine.await(drivetrain.driveDistance(1.0));
        })
        .named("Drive Turn Drive");
    }
}
