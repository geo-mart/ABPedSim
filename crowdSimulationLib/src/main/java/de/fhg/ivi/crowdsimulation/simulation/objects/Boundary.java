package de.fhg.ivi.crowdsimulation.simulation.objects;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

/**
 * A {@link Boundary} consists on a {@link Geometry}, which could be e.g. points, lines or polygons,
 * which are obstacles for the movement of the {@link Pedestrian}s.
 * <p>
 * Based on this a {@link Boundary} is a repulsive force in case of the Social Force Model. In the
 * formula of Helbing et al. (2005) this is represented through the second term of the common
 * formula. Look at p. 11, 12 and formula 3, 7 (in this publication) for understanding.
 * <p>
 * Otherwise this class encapsulates the {@link Geometry} object and the caches the bounding box of
 * it for fast access.
 *
 * @author hahmann/meinert
 */
public class Boundary
{
    /**
     * The {@link Geometry} object of this {@link Boundary}.
     */
    private Geometry geometry;

    /**
     * Caches {@link Geometry#getEnvelopeInternal()} of {@link Boundary#geometry} expanded by the
     * maximum distance of interaction between a {@link Pedestrian} and a {@link Boundary}
     */
    private Envelope boundingBox;

    /**
     * Creates a new {@link Geometry} of a {@link Boundary} object.
     * <p>
     * Otherwise computes an {@link Envelope}, which based on the {@link Boundary} plus a specific
     * {@code maxBoundaryInteractionDistance}.
     *
     * @param geometry defines the outline of the {@link Boundary}
     * @param maxBoundaryInteractionDistance maximum distance of interaction between a
     *            {@link Pedestrian} and a {@link Boundary}
     */
    public Boundary(Geometry geometry, double maxBoundaryInteractionDistance)
    {
        this.geometry = geometry;
        this.boundingBox = geometry.getEnvelopeInternal();
        this.boundingBox.expandBy(maxBoundaryInteractionDistance);
    }

    /**
     * Gets the {@link Geometry} of this {@link Boundary} object.
     *
     * @return the {@link Geometry} of this {@link Boundary}
     */
    public Geometry getGeometry()
    {
        return geometry;
    }

    /**
     * Gets a cached version of {@link Geometry#getEnvelopeInternal()} of {@link Boundary#geometry}
     * expanded by the maximum distance of interaction between a {@link Pedestrian} and a
     * {@link Boundary}.
     *
     * @return a cached version of {@link Geometry#getEnvelopeInternal()} of
     *         {@link Boundary#geometry} expanded by the maximum distance of interaction between a
     *         {@link Pedestrian} and a {@link Boundary}
     */
    public Envelope getBoundingBox()
    {
        return boundingBox;
    }
}
