package simulation.trajectory;

import control.Trajectory1d;
import control.Trajectory4d;

/**
 * Forwarding decorator for 4D Trajectories.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class Trajectory4DForwardingDecorator {
    private final Trajectory4d target;

    public Trajectory4DForwardingDecorator(Trajectory4d target) {
        this.target = target;
    }

    public Trajectory1d getTrajectoryLinearX() {
        return getTrajectory4DDelegate().getTrajectoryLinearX();
    }

    public Trajectory1d getTrajectoryLinearY() {
        return getTrajectory4DDelegate().getTrajectoryLinearY();
    }

    public Trajectory1d getTrajectoryLinearZ() {
        return getTrajectory4DDelegate().getTrajectoryLinearZ();
    }

    public Trajectory1d getTrajectoryAngularZ() {
        return getTrajectory4DDelegate().getTrajectoryAngularZ();
    }

    protected Trajectory4d getTrajectory4DDelegate() {
        return target;
    }
}
