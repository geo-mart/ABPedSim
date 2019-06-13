package de.fhg.ivi.crowdsimulation.simulation.tools;

import java.awt.Shape;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opensphere.geometry.algorithm.ConcaveHull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.TopologyException;
import com.vividsolutions.jts.operation.linemerge.LineMerger;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import math.geom2d.Vector2D;

/**
 * This class represents a collection of static methods for geometry-related calculations. There are
 * several functions implemented in this class:
 *
 * <li>Calculation of the nearest {@link Coordinate} on a {@link Geometry}</li>
 * <li>Calculation of a perpendicular line based on two {@link Coordinate} and a width</li>
 * <li>Sorting of a {@link Map} in dependence to a {@link Double} value</li>
 * <li>Computes a normalized {@link Vector2D} between a {@link Geometry} and a
 * {@link Coordinate}</li>
 * <li>Calculation of a normalized version of a {@link Vector2D}</li>
 * <li>Convert of a {@link List} of {@link Geometry}s to a {@link List} of {@link Shape}s</li>
 * <li>Tests whether a {@link Vector2D} lies inside an {@link Envelope}</li>
 * <li>Tests whether a bounding box of two {@link Vector2D}s intersects an {@link Envelope}</li>
 * <li>Creating an convex or concave outline around a set of {@link MultiPoint}s</li>
 * <li>Extract a {@link List} of {@link Coordinate}s out of a {@link List} of
 * {@link Pedestrian}s</li>
 *
 * <p>
 *
 * @author hahmann/meinert
 */
public class GeometryTools
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(GeometryTools.class);

    /**
     * Calculates a {@link Coordinate} on a {@link Geometry} object based on the shortest distance
     * between a given {@link Coordinate} and a given {@link Geometry}.
     *
     * @param coordinate a {@link Coordinate} which describes a x, y position
     * @param geometry a {@link Geometry}
     *
     * @return a {@link Coordinate} representing the point on a {@code geometry} that is nearest to
     *         {@code coordinate} or {@code null} if no such point has been found.
     */
    public static Coordinate getNearestPoint(Coordinate coordinate, Geometry geometry)
    {
        Coordinate[] coordinates = geometry.getCoordinates();

        // null geometry or empty geometry
        if (coordinates == null || coordinates.length == 0)
        {
            return null;
        }
        // trivial case of a point
        else if (coordinates.length == 1)
        {
            return coordinates[0];
        }
        else
        {
            double distanceSquaredClosestCoordinate = Double.MAX_VALUE;
            Coordinate closestCoordinate = null;

            // loop over lineSegments from a Polygon/LineString
            for (int i = 0; i < coordinates.length - 1; i++ )
            {
                LineSegment lineSegment = new LineSegment(coordinates[i], coordinates[i + 1]);
                // computes closest point in dependence of LineSegment
                Coordinate closestCoordinateCandidate = lineSegment.closestPoint(coordinate);

                // computes squared distance between lineSegment and point
                double distanceSquared = MathTools.distanceSquared(closestCoordinateCandidate,
                    coordinate);
                if (distanceSquared >= distanceSquaredClosestCoordinate)
                {
                    continue;
                }

                distanceSquaredClosestCoordinate = distanceSquared;
                closestCoordinate = closestCoordinateCandidate;
            }

            if (closestCoordinate != null)
            {
                return closestCoordinate;
            }
            return null;
        }
    }

    /**
     * Calculates a perpendicular line based on two {@link Coordinate}s. Afterwards the line will be
     * rotated around 90 degrees. The width of this line is limited by the {@code width}.
     *
     * @param firstPoint is the first {@link Coordinate}, which is the starting point of the
     *            perpendicular line
     * @param lastPoint is a {@link Coordinate}, which is a reference point for the perpendicular
     *            line
     * @param width denotes the width of the perpendicular line
     *
     * @return a perpendicular line as {@link Geometry}
     */
    public static Geometry getPerpendicularLine(Coordinate firstPoint, Coordinate lastPoint,
        double width)
    {
        com.vividsolutions.jts.math.Vector2D directionVector = new com.vividsolutions.jts.math.Vector2D(
            firstPoint, lastPoint);
        // rotate with 90 degrees + set length to specified meters
        com.vividsolutions.jts.math.Vector2D perpendicularVector90 = directionVector.normalize()
            .multiply(width)
            .rotateByQuarterCircle(1);
        // rotate with 270 degrees + set length to specified meters
        com.vividsolutions.jts.math.Vector2D perpendicularVector270 = directionVector.normalize()
            .multiply(width)
            .rotateByQuarterCircle(3);

        // create coordinate[] for creating LineStrings
        Coordinate[] rightLineCoordinates = new Coordinate[] { firstPoint,
            new Coordinate(firstPoint.x + perpendicularVector90.getX(),
                firstPoint.y + perpendicularVector90.getY()) };
        Coordinate[] leftLineCoordinates = new Coordinate[] { firstPoint,
            new Coordinate(firstPoint.x + perpendicularVector270.getX(),
                firstPoint.y + perpendicularVector270.getY()) };

        // create lineStrings in each direction
        LineString rightLine = JTSFactoryFinder.getGeometryFactory()
            .createLineString(rightLineCoordinates);
        LineString leftLine = JTSFactoryFinder.getGeometryFactory()
            .createLineString(leftLineCoordinates);

        // merge the 90 degrees line and the 270 degrees line to one line
        LineMerger lineMerger = new LineMerger();
        lineMerger.add(rightLine);
        lineMerger.add(leftLine);
        Collection<LineString> mergedLineStrings = lineMerger.getMergedLineStrings();
        GeometryCollection gc = new GeometryCollection(mergedLineStrings.toArray(new Geometry[] {}),
            JTSFactoryFinder.getGeometryFactory());
        Geometry perpendicularLine = gc.union();

        return perpendicularLine;
    }

    /**
     * Sorts the {@link Coordinate}s in this {@link Map} depending on the smallest {@link Double}
     * value.
     *
     * @param map contains all {@link Coordinate}s, as key, and associated distances to a
     *            {@link Pedestrian}.
     *
     * @return a new map, with the same declaration, which is sorted by the {@link Double} value
     */
    public static List<Map.Entry<Coordinate, Double>> sortMap(Map<Coordinate, Double> map)
    {
        List<Map.Entry<Coordinate, Double>> list = new LinkedList<>(map.entrySet());
        Collections.sort(list, new Comparator<Map.Entry<Coordinate, Double>>()
            {
                @Override
                public int compare(Map.Entry<Coordinate, Double> pointA,
                    Map.Entry<Coordinate, Double> pointB)
                {
                    return (pointA.getValue()).compareTo(pointB.getValue());
                }
            });
        return list;
    }

    /**
     * Computes the normalized vector based on the position of a {@link Geometry} and a
     * {@link Coordinate}..
     *
     * @param currentPosition the {@link Coordinate} object which describes a x, y position
     * @param geometry a {@link Geometry} object
     *
     * @return the {@link Vector2D} object of the normalized direction vector of the geometry
     */
    public static Vector2D getEVectorOfBoundary(Coordinate currentPosition, Geometry geometry)
    {
        Coordinate closestCoordinate = getNearestPoint(currentPosition, geometry);
        Coordinate[] coordinates = geometry.getCoordinates();
        int coordinateIndex = 0;
        Vector2D eBeta;

        // null geometry or empty geometry
        if (coordinates == null || coordinates.length == 0)
        {
            return null;
        }
        // trivial case of a point
        else if (coordinates.length == 1)
        {
            eBeta = MathTools.normalize(coordinates[0].x, coordinates[0].y);

            return eBeta;
        }
        else
        {
            for (int i = 0; i < coordinates.length - 1; i++ )
            {
                if (closestCoordinate == coordinates[i])
                {
                    coordinateIndex = i;
                    break;
                }
            }

            // TODO, phi = Skalarprodukt der normierten Richtungsvektoren
            // eAlpha.normalize().dot(eBeta.normalize());
            // Normalisierung des e Vektors des P und des e Vektors der Geometrie -> das
            // Skalarprodukt
            // ergibt dann den kürzeren
            // der kürzere ist relevant da der P ja im <90 Grad Winkel auf die Geometrien trifft
            // ich denke das gehört bei geometryTools in die funktion - dann mal schauen
            // mindestens teilweise für die phi Berechnung genutzt wurd
            // weiterhin phi Berechnung gegen das aus Helbing Paper ändern (siehe Zettel)

            Vector2D firstGeomPoint = new Vector2D(coordinates[coordinateIndex].x,
                coordinates[coordinateIndex].y);
            Vector2D secondeGeomPoint = new Vector2D(coordinates[coordinateIndex + 1].x,
                coordinates[coordinateIndex + 1].y);
            Vector2D thirdGeomPoint = null;

            if (coordinateIndex > 0)
            {
                thirdGeomPoint = new Vector2D(coordinates[coordinateIndex - 1].x,
                    coordinates[coordinateIndex - 1].y);
            }

            logger.debug("GeometryTools.getEVectorOfBoundary(), firstGeomPoint: " + firstGeomPoint);
            logger.debug(
                "GeometryTools.getEVectorOfBoundary(), secondeGeomPoint: " + secondeGeomPoint);
            logger.debug("GeometryTools.getEVectorOfBoundary(), thirdGeomPoint: " + thirdGeomPoint);
            logger.trace("GeometryTools.getEVectorOfBoundary(), coordinates[coordinateIndex]: "
                + coordinates[coordinateIndex]);

            Vector2D eBetaA = secondeGeomPoint.minus(firstGeomPoint)
                .times(1 / MathTools.norm(secondeGeomPoint.minus(firstGeomPoint)));

            Vector2D eBetaB = null;
            if (thirdGeomPoint != null)
            {
                eBetaB = thirdGeomPoint.minus(firstGeomPoint)
                    .times(1 / (MathTools.norm(thirdGeomPoint.minus(firstGeomPoint))));
            }

            logger.debug("GeometryTools.getEVectorOfBoundary(), eBetaA: " + eBetaA);
            logger.debug("GeometryTools.getEVectorOfBoundary(), eBetaB: " + eBetaB);

            eBeta = eBetaA;

            return eBeta;
        }
    }

    /**
     * Converts a {@link List} of {@link Geometry}s into a {@link List} of {@link Shape}s.
     *
     * @param wayPointVerticals is a {@link List} of {@link Geometry}s
     *
     * @return a {@link List} of {@link Shape}s
     */
    public static List<Shape> convertGeometriesToShapes(List<Geometry> wayPointVerticals)
    {
        List<Shape> shapes = new ArrayList<>();
        ShapeWriter sw = new ShapeWriter();
        for (Geometry wayPointVertical : wayPointVerticals)
        {
            shapes.add(sw.toShape(wayPointVertical));
        }

        return shapes;
    }

    /**
     * Tests whether a {@code vector} is inside a {@code boundingBox}.
     *
     * @param vector is a {@link Vector2D}, which defines a x, y position
     * @param boundingBox is an {@link Envelope} object
     *
     * @return {@code true} if the {@code vector} lying inside the {@code boundingBox}. If not it's
     *         {@code false}.
     */
    public static boolean isInside(Vector2D vector, Envelope boundingBox)
    {
        if (vector == null || boundingBox == null)
        {
            return false;
        }
        return boundingBox.getMinX() <= vector.x() && boundingBox.getMaxX() >= vector.x()
            && boundingBox.getMinY() <= vector.y() && boundingBox.getMaxY() >= vector.y();
    }

    /**
     * Tests whether a bounding box defined by two {@link Vector2D}s intersects a given
     * {@link Envelope}.
     *
     * @param vector1 is a {@link Vector2D}, which defines a x, y position
     * @param vector2 is a {@link Vector2D}, which defines a x, y position
     * @param boundingBox the {@link Envelope} object
     *
     * @return
     *         <li>{@code false} if {@code vector1} is {@code null}, {@code vector2} is {@code null}
     *         or {@code BoundingBox} is {@code null}
     *         <li>{@code true} if the bounding box given by {@code vector1} and {@code vector2}
     *         intersects {@code boundingBox},
     *         <li>{@code false} otherwise
     */
    public static boolean intersects(Vector2D vector1, Vector2D vector2, Envelope boundingBox)
    {
        if (vector1 == null || vector2 == null || boundingBox == null)
        {
            return false;
        }
        return !(boundingBox.getMinX() > Math.max(vector1.x(), vector2.x())
            || boundingBox.getMaxX() < Math.min(vector1.x(), vector2.x())
            || boundingBox.getMinY() > Math.max(vector1.y(), vector2.y())
            || boundingBox.getMaxY() < Math.min(vector1.y(), vector2.y()));
    }

    /**
     * Creates the outline as {@link Geometry} of a given {@link MultiPoint} {@code multiPoint}
     * object, the boolean {@code isConvex} flag and the {@code concaveCrowdOutlineThreshold} (only
     * required for {@code isConvex} equals {@code false}
     *
     * @param multiPoint {@link MultiPoint} to create an outline for
     * @param boundaries a {@link Geometry} object to intersect the resulting outline object with
     * @param isConvex flag to decide if the outline should be convex or concave
     * @param concaveCrowdOutlineThreshold a threshold to be used in the {@link ConcaveHull}
     *            algorithm (cf. Duckham et al. 2008). "For n points, concaveCrowdOutlineThreshold
     *            denotes the largest threshold distance such that all n points can still fit inside
     *            the shape"
     *
     * @return an outline of the {@code multiPoint} as a {@link Geometry}
     */
    public static Geometry createOutline(MultiPoint multiPoint, Geometry boundaries,
        boolean isConvex, double concaveCrowdOutlineThreshold)
    {
        Geometry outline = null;
        if (isConvex)
        {
            Geometry convexOutline = multiPoint.convexHull();
            if (boundaries != null && !boundaries.isEmpty())
                outline = convexOutline.difference(boundaries);
            else
                outline = convexOutline;
        }
        else
        {
            try
            {
                Geometry concaveCrowdOutline = new ConcaveHull(multiPoint,
                    concaveCrowdOutlineThreshold).getConcaveHull();
                // update and intersect crowd outline with boundaries, if boundaries already exist
                if (concaveCrowdOutline != null && concaveCrowdOutline instanceof Polygon
                    && concaveCrowdOutline.isValid() && boundaries != null && !boundaries.isEmpty())
                    outline = concaveCrowdOutline.difference(boundaries);
                else
                    outline = concaveCrowdOutline;
            }
            catch (IndexOutOfBoundsException | TopologyException e)
            {
                Geometry convexHull = multiPoint.convexHull();
                if (convexHull != null && convexHull instanceof Polygon && convexHull.isValid()
                    && boundaries != null && !boundaries.isEmpty())
                {
                    outline = convexHull.difference(boundaries);
                }
                else
                {
                    outline = convexHull;
                }
                logger.debug("GeometryTools.createOutline(), ", e);
            }
        }
        if (outline != null && outline instanceof LineString)
        {
            outline = outline.buffer(0);
        }
        if (outline == null)
            logger.debug("GeometryTools.createOutline(), outline==null");

        return outline;
    }

    /**
     * Gets the x and y position of every {@link Pedestrian} out of a {@link List} of
     * {@link Pedestrian}s and collects them in a {@link Coordinate}[] object.
     *
     * @return a {@link Coordinate}[] object with all positions of all {@link Pedestrian}
     */
    public static Coordinate[] getCoordinatesFromPedestrians(List<Pedestrian> pedestrians)
    {
        Coordinate[] coordinates = null;
        synchronized (pedestrians)
        {
            coordinates = new Coordinate[pedestrians.size()];
            for (int i = 0; i < coordinates.length; i++ )
            {
                Vector2D currentPedestrianPostion = pedestrians.get(i).getCurrentPosition();
                coordinates[i] = new Coordinate(currentPedestrianPostion.x(),
                    currentPedestrianPostion.y());
            }
        }

        return coordinates;
    }

    /**
     * Creates a {@link Point} from a given {@link Coordinate} object
     *
     * @param c the {@link Coordinate} object
     * @return the {@link Point} object
     */
    public static Point coordinateToPoint(Coordinate c)
    {
        return JTSFactoryFinder.getGeometryFactory().createPoint(c);
    }

    /**
     * Converts a {@link List} of {@link Boundary} objects to a {@link GeometryCollection} object
     *
     * @param listOfAllBoundaries the {@link List} of {@link Boundary} objects
     * @return the converted {@link GeometryCollection} object
     */
    public static GeometryCollection boundariesToGeometryCollection(
        List<Boundary> listOfAllBoundaries)
    {
        Boundary[] boundariesArray = listOfAllBoundaries.toArray(new Boundary[0]);
        Geometry[] geometriesArray = new Geometry[boundariesArray.length];
        for (int i = 0; i < geometriesArray.length; i++ )
        {
            geometriesArray[i] = boundariesArray[i].getGeometry();
        }
        GeometryCollection collectionOfAllBoundaries = JTSFactoryFinder.getGeometryFactory()
            .createGeometryCollection(geometriesArray);
        return collectionOfAllBoundaries;
    }

    /**
     * TODO
     *
     * @param wayPoints
     * @return
     */
    public static List<Geometry> wayPointsToGeometries(List<WayPoint> wayPoints)
    {
        if (wayPoints == null || wayPoints.isEmpty())
            return null;
        List<Geometry> wayPointsAsGeometries = new ArrayList<>();
        for (WayPoint wayPoint : wayPoints)
        {
            wayPointsAsGeometries.add(wayPoint.toGeometry());

        }
        return wayPointsAsGeometries;
    }

    /**
     * Checks whether there is a direct line of sight between between start and target
     * {@link Coordinate}s or whether this is blocked through a {@code boundaries}.
     *
     * @param start start {@link Coordinate}
     * @param target target {@link Coordinate}
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @return {@code true} if there is a direct line of sight between start and target,
     *         {@code false} otherwise
     * 
     *         Code similar to method in FollowWayPointsMentalModel class of mentalmodal package
     */
    public static boolean isTargetVisible(Coordinate start, Coordinate target,
        List<Boundary> boundaries)
    {

        // computes LineString between start and target
        Coordinate[] coordinates = new Coordinate[] { start, target };
        LineString lineOfSight = JTSFactoryFinder.getGeometryFactory()
            .createLineString(coordinates);

        // checks if the Pedestrian sees the its next Waypoint.
        boolean lineOfSightIntersectsBoundary = false;
        if (boundaries != null)
        {
            Envelope boundingBoxLineOfSight = lineOfSight.getEnvelopeInternal();
            for (Boundary boundary : boundaries)
            {
                // quick check
                if (boundingBoxLineOfSight.intersects(boundary.getBoundingBox()))
                {
                    // accurate check
                    lineOfSightIntersectsBoundary = lineOfSight.intersects(boundary.getGeometry());
                    if (lineOfSightIntersectsBoundary)
                        break;
                }
            }
        }
        return !lineOfSightIntersectsBoundary;
    }

    /**
     * Converts real world coordinates to java world coordinates
     *
     * @param geometry given {@link Geometry}
     * @return {@link Geometry} in java coordinate system
     *
     *         Code similar to method in DataTools class of ui package
     */
    public static Geometry transformGeometry(Geometry geometry)
    {
        if (geometry == null)
            return null;
        MathTransform transform = new org.geotools.referencing.operation.transform.AffineTransform2D(
            1, 0, 0, -1, 0, 0);
        Geometry transformedGeometry = null;
        try
        {
            transformedGeometry = JTS.transform(geometry, transform);
        }
        catch (MismatchedDimensionException | TransformException e)
        {
            logger.error("transformGeometries(), error when transforming data", e);
        }

        return transformedGeometry;
    }

}
