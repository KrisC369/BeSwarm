package applications.parrot.tumsim;

import applications.trajectory.TrajectoryServer;
import control.FiniteTrajectory4d;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.reflections.Reflections;
import org.slf4j.LoggerFactory;

import java.util.Collection;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.fail;

/**
 * Initialization tests for TumSim example files.
 * This test uses reflection to scan the tumsim package for extensions of the
 * AbstractTumSimulatorExample and runs initialization tests on instances of the found classes.
 * These tests validate correct parameters passed to trajectory creators before attempting to use
 * such trajectories in simulation or real world experiments.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@RunWith(Parameterized.class)
public class TumSimulatorExampleTest {
    private TrajectoryServer server;

    public TumSimulatorExampleTest(Class cl) {
        try {
            this.server = (TrajectoryServer) cl.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            fail("Instantiation with default constructor should be possible for "
                    + "TumSimulatorExamples");
        }
    }

    @Parameterized.Parameters
    public static Collection<? extends Class> getData() {
        Reflections reflections = new Reflections("applications.parrot.tumsim");
        return reflections.getSubTypesOf(AbstractTumSimulatorExample.class) s;
    }

    @Test
    public void testTrajectoryInitialization() {
        LoggerFactory.getLogger(TumSimulatorExampleTest.class)
                .info("Running example initialization test for instances of " + server.getClass()
                        .getName());
        FiniteTrajectory4d traj = server.getConcreteTrajectory();
        assertNotEquals(0, traj.getTrajectoryDuration());
    }

}