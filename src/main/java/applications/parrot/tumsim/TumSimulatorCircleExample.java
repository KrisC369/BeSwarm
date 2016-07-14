package applications.parrot.tumsim;

import applications.trajectory.Trajectories;
import applications.trajectory.points.Point3D;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import control.Trajectory4d;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class TumSimulatorCircleExample extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("TumRunExampleTrajectory2");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final FiniteTrajectory4d trajectory = getConcreteTrajectory();
        final TumExampleFlightFacade flight = TumExampleFlightFacade.create(trajectory, connectedNode);
        flight.fly();
    }

    public static FiniteTrajectory4d getConcreteTrajectory() {
        //Alternatively, the builder api can also be used like this, to create circle trajectory.
        Trajectory4d second = Trajectories.circleTrajectoryBuilder()
                .setLocation(Point3D.create(1, -2, 1.5))
                .setRadius(0.5)
                .setFrequency(0.05)
                .fixYawAt(-Math.PI / 2)
                .build();
        return Choreography.builder()
                .withTrajectory(second)
                .forTime(120)
                .build();
    }
}
