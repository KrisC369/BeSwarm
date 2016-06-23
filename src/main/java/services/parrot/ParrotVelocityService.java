package services.parrot;

import control.dto.Pose;
import geometry_msgs.Twist;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import control.dto.Velocity;
import services.VelocityService;
import utils.math.Transformations;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 * A facade which provides an interface to publish velocity (a.k.a. piloting) message. There are minimum and maximum
 * values for each dimension of the velocity. If the input in one dimension of the velocity is out of the min/max
 * range, the dimension will be set to the min/max value.
 *
 * @author Hoang Tung Dinh
 */
public final class ParrotVelocityService implements VelocityService {
    private final Publisher<Twist> publisher;

    private final double minLinearX;
    private final double minLinearY;
    private final double minLinearZ;
    private final double minAngularZ;

    private final double maxLinearX;
    private final double maxLinearY;
    private final double maxLinearZ;
    private final double maxAngularZ;

    private ParrotVelocityService(Builder builder) {
        publisher = builder.publisher;
        minLinearX = builder.minLinearX;
        maxLinearX = builder.maxLinearX;
        minLinearY = builder.minLinearY;
        maxLinearY = builder.maxLinearY;
        minLinearZ = builder.minLinearZ;
        maxLinearZ = builder.maxLinearZ;
        minAngularZ = builder.minAngularZ;
        maxAngularZ = builder.maxAngularZ;
        checkArgument(publisher.getTopicName().toString().endsWith("/cmd_vel"),
                "Topic name must be [namespace]/cmd_vel");
        checkArgument(minLinearX <= maxLinearX);
        checkArgument(minLinearY <= maxLinearY);
        checkArgument(minLinearZ <= maxLinearZ);
    }


    /**
     * Publish a velocity message.
     *
     * @param velocity the velocity to be published
     */
    @Override
    public void sendVelocityMessage(Velocity velocity) {
        final Pose pose = velocity.pose();
        checkNotNull(pose, "The pose must not be null.");
        final Velocity velocityInLocalFrame = Transformations.globalVelocityToLocalVelocity(
                velocity, velocity.pose().yaw());

        final Velocity refinedVelocity = getRefinedVelocity(velocityInLocalFrame);
        
        publisher.publish(newTwistMessage(refinedVelocity));
    }

    private Twist newTwistMessage(Velocity refinedVelocity) {
    	final Twist twist = publisher.newMessage();
        twist.getLinear().setX(refinedVelocity.linearX());
        twist.getLinear().setY(refinedVelocity.linearY());
        twist.getLinear().setZ(refinedVelocity.linearZ());
        twist.getAngular().setZ(refinedVelocity.angularZ());
        
        return twist;
    }
    
    private Velocity getRefinedVelocity(Velocity velocity) {
        return Velocity.builder()
                .linearX(getRefinedLinearX(velocity.linearX()))
                .linearY(getRefinedLinearY(velocity.linearY()))
                .linearZ(getRefinedLinearZ(velocity.linearZ()))
                .angularZ(getRefinedAngularZ(velocity.angularZ()))
                .build();
    }

    private double getRefinedLinearX(double linearX) {
        return getRefinedValue(linearX, minLinearX, maxLinearX);
    }

    private double getRefinedLinearY(double linearY) {
        return getRefinedValue(linearY, minLinearY, maxLinearY);
    }

    private double getRefinedLinearZ(double linearZ) {
        return getRefinedValue(linearZ, minLinearZ, maxLinearZ);
    }

    private double getRefinedAngularZ(double angularZ) {
        return getRefinedValue(angularZ, minAngularZ, maxAngularZ);
    }

    private double getRefinedValue(double value, double minValue, double maxValue) {
        double refinedValue = value;

        if (value > maxValue) {
            refinedValue = maxValue;
        } else if (refinedValue < minValue) {
            refinedValue = minValue;
        }

        return refinedValue;
    }

    /**
     * The {@link Builder#publisher(Publisher)} is mandatory.
     * All min and max values are optional.
     *
     * @return a builder instance of this facade
     */
    public static Builder builder() {
        return new Builder().minLinearX(-Double.MAX_VALUE)
                .minLinearY(-Double.MAX_VALUE)
                .minLinearZ(-Double.MAX_VALUE)
                .minAngularZ(-Double.MAX_VALUE)
                .maxLinearX(Double.MAX_VALUE)
                .maxLinearY(Double.MAX_VALUE)
                .maxLinearZ(Double.MAX_VALUE)
                .maxAngularZ(Double.MAX_VALUE);
    }
    
    public static Builder createServiceBuilder(String droneName, ConnectedNode connectedNode) {
    	return ParrotVelocityService.builder().publisher(connectedNode.<Twist>newPublisher(droneName + "cmd/vel", Twist._TYPE));
    }

    /**
     * {@code VelocityPublisher} builder static inner class.
     */
    public static final class Builder {
        private Publisher<Twist> publisher;

        private double minLinearX;
        private double minLinearY;
        private double minLinearZ;
        private double minAngularZ;

        private double maxLinearX;
        private double maxLinearY;
        private double maxLinearZ;
        private double maxAngularZ;

        private Builder() {}

        /**
         * Sets the {@code publisher} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code publisher} to set
         * @return a reference to this Builder
         */
        public Builder publisher(Publisher<Twist> val) {
            publisher = val;
            return this;
        }

        /**
         * Sets the {@code minLinearX} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code minLinearX} to set
         * @return a reference to this Builder
         */
        public Builder minLinearX(double val) {
            minLinearX = val;
            return this;
        }

        /**
         * Sets the {@code maxLinearX} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code maxLinearX} to set
         * @return a reference to this Builder
         */
        public Builder maxLinearX(double val) {
            maxLinearX = val;
            return this;
        }

        /**
         * Sets the {@code minLinearY} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code minLinearY} to set
         * @return a reference to this Builder
         */
        public Builder minLinearY(double val) {
            minLinearY = val;
            return this;
        }

        /**
         * Sets the {@code maxLinearY} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code maxLinearY} to set
         * @return a reference to this Builder
         */
        public Builder maxLinearY(double val) {
            maxLinearY = val;
            return this;
        }

        /**
         * Sets the {@code minLinearZ} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code minLinearZ} to set
         * @return a reference to this Builder
         */
        public Builder minLinearZ(double val) {
            minLinearZ = val;
            return this;
        }

        /**
         * Sets the {@code maxLinearZ} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code maxLinearZ} to set
         * @return a reference to this Builder
         */
        public Builder maxLinearZ(double val) {
            maxLinearZ = val;
            return this;
        }

        /**
         * Sets the {@code minAngularZ} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code minAngularZ} to set
         * @return a reference to this Builder
         */
        public Builder minAngularZ(double val) {
            minAngularZ = val;
            return this;
        }

        /**
         * Sets the {@code maxAngularZ} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code maxAngularZ} to set
         * @return a reference to this Builder
         */
        public Builder maxAngularZ(double val) {
            maxAngularZ = val;
            return this;
        }

        /**
         * Returns a {@code VelocityPublisher} built from the parameters previously set.
         *
         * @return a {@code VelocityPublisher} built with parameters of this {@code VelocityPublisher.Builder}
         */
        public ParrotVelocityService build() {
            return new ParrotVelocityService(this);
        }
    }
}
