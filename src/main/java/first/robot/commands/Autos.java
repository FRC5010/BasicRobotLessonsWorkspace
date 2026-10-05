package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Function;

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

    public static Command driveTurnDriveCoroutine(Drivetrain drivetrain, double meters){
        return Command.noRequirements(coroutine -> {
            for (int i = 0; i<4; i++){
            coroutine.await(drivetrain.driveDistance(meters));
            coroutine.wait(Seconds.of(1.0));
            coroutine.await(drivetrain.turnToHeading(90));
            }
        })
        .named("Drive Turn Drive");
    }
}
