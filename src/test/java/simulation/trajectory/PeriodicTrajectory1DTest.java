package simulation.trajectory;

import com.google.common.collect.Lists;
import control.Trajectory1d;
import control.Trajectory2d;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.util.Collection;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@RunWith(value = Parameterized.class)
public class PeriodicTrajectory1DTest {
    private Trajectory1d highFrequencyCircle;
    private Trajectory1d lowFrequencyCircle;
    private final double lowFreq = 1 / 10;
    private final double highFreq = 1.5;
    private final double radius = 0.065;

    public PeriodicTrajectory1DTest(Class cl) {
        Class[] cArg = new Class[2];
        cArg[0] = double.class;
        cArg[1] = double.class;
        try {
            highFrequencyCircle = (Trajectory1d) cl.getDeclaredConstructor(cArg)
                    .newInstance(radius, highFreq);
            lowFrequencyCircle = (Trajectory1d) cl.getDeclaredConstructor(cArg)
                    .newInstance(radius, lowFreq);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Parameterized.Parameters
    public static Collection<? extends Class> getParams() {
        return Lists.newArrayList(ConstantSwingTrajectory1D.class,
                PendulumSwingTrajectory1D.class);
    }

    @Test
    public void getTrajectoryPosition() throws Exception {
        for (double i = 0; i < 3;
             i += 0.66) {
            Assert.assertEquals(radius,
                    highFrequencyCircle
                            .getDesiredPosition(i), 0.01);
        }

        for (double i = 0; i < 30;
             i += 10) {
            Assert.assertEquals(radius,
                    lowFrequencyCircle
                            .getDesiredPosition(i), 0.01);
        }
    }

    @Test
    public void getTrajectoryVelocity() {
        for (double i = 0; i < 30;
             i += 2) {

            Assert.assertTrue(
                    Math.abs(highFrequencyCircle
                            .getDesiredVelocity(i))
                            < PeriodicTrajectory.MAX_ABSOLUTE_SPEED);
        }

        for (double i = 0; i < 30;
             i += 2) {

            Assert.assertTrue(
                    Math.abs(lowFrequencyCircle
                            .getDesiredVelocity(i))
                            < PeriodicTrajectory.MAX_ABSOLUTE_SPEED);
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void testConstructorWithHighThanAllowedSpeedRate() {
        Trajectory2d target = new CircleTrajectory2D(5, 1, true);
    }

}