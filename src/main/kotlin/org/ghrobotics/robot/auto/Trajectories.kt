/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot.auto

/* ktlint-disable no-wildcard-imports */
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derivedunits.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.robot.subsytems.drive.DriveSubsystem

/**
 * Contains trajectories for autonomous
 */
object Trajectories {

    // Constants in Feet Per Second
    private val kMaxVelocity = 10.0.feet.velocity // TODO Find Actual Value
    private val kMaxAcceleration = 5.0.feet.acceleration // TODO Find Actual Value
    private val kMaxCentripetalAcceleration = 4.5.feet.acceleration // TODO Find Actual Value

    // Constraints
    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(DriveSubsystem.driveModel, 10.0.volt)
    )

    private fun waypoints(vararg waypoints: Pose2d) = listOf(*waypoints)

    private fun List<Pose2d>.generateTrajectory(
        name: String,
        reversed: Boolean,
        maxVelocity: Velocity = kMaxVelocity,
        maxAcceleration: Acceleration = kMaxAcceleration,
        constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints
    ): TimedTrajectory<Pose2dWithCurvature> {
        println("Generating $name...")
        return DefaultTrajectoryGenerator.generateTrajectory(
            reversed = reversed,
            wayPoints = this,
            constraints = constraints,
            startVelocity = 0.meter.velocity,
            endVelocity = 0.meter.velocity,
            maxVelocity = maxVelocity,
            maxAcceleration = maxAcceleration
        )
    }
}