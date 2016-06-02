package geom;

import geometry_msgs.Quaternion;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.util.Collection;

import static com.google.common.truth.Truth.assertThat;
import static java.util.Arrays.asList;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/**
 * @author Hoang Tung Dinh
 */
@RunWith(Parameterized.class)
public class EulerAngleTest {
    private final double quaternionW;
    private final double quaternionX;
    private final double quaternionY;
    private final double quaternionZ;

    private final double eulerX;
    private final double eulerY;
    private final double eulerZ;

    private static final double DELTA = 0.01;

    public EulerAngleTest(double quaternionW, double quaternionX, double quaternionY, double quaternionZ, double eulerX,
            double eulerY, double eulerZ) {
        this.quaternionW = quaternionW;
        this.quaternionX = quaternionX;
        this.quaternionY = quaternionY;
        this.quaternionZ = quaternionZ;
        this.eulerX = eulerX;
        this.eulerY = eulerY;
        this.eulerZ = eulerZ;
    }

    @Parameterized.Parameters
    public static Collection<Object[]> data() {
        // (w x y z) (x y z)
        return asList(new Object[][]{{1, 0, 0, 0, 0, 0, 0},
                {0.7071, 0, 0.7071, 0, 0, 1.5707963267948966, 0},
//                {0, 0, 1, 0, 0, 3.141592653589793, 0},
                {0.7071, 0, -0.7071, 0, 0, -1.5707963267948966, 0}});
    }

    @Test
    public void testComputeEulerAngleFromQuaternionAngle() {
        // TODO improve this test
        final Quaternion mockQuaternion = mock(Quaternion.class);
        when(mockQuaternion.getW()).thenReturn(quaternionW);
        when(mockQuaternion.getX()).thenReturn(quaternionX);
        when(mockQuaternion.getY()).thenReturn(quaternionY);
        when(mockQuaternion.getZ()).thenReturn(quaternionZ);

        final EulerAngle eulerAngle = EulerAngle.createFromQuaternion(mockQuaternion);

        assertThat(eulerAngle.angleX()).isWithin(DELTA).of(eulerX);
        assertThat(eulerAngle.angleY()).isWithin(DELTA).of(eulerY);
        assertThat(eulerAngle.angleZ()).isWithin(DELTA).of(eulerZ);
    }
}