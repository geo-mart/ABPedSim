package de.fhg.ivi.crowdsimulation.ui.gui.tools;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.io.FilenameUtils;
import org.geotools.data.DataStore;
import org.geotools.data.DataStoreFinder;
import org.geotools.data.FeatureSource;
import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.JTS;
import org.geotools.graph.build.feature.FeatureGraphGenerator;
import org.geotools.graph.build.line.LineStringGraphGenerator;
import org.geotools.graph.structure.Graph;
import org.geotools.referencing.CRS;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.simple.SimpleFeatureType;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.csvreader.CsvReader;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.io.ParseException;
import com.vividsolutions.jts.io.WKTReader;

import hcu.csl.agentbasedmodeling.PedestrianAgent;

/**
 * Class for providing helping methods in case of data-processing use cases. There are several
 * functions implemented in this class:
 *
 * <li>Analysis whether a {@link File} has a "csv" or "shp" format</li>
 * <li>Extraction of a {@link Geometry} out of a "csv" {@link File}</li>
 * <li>Extraction of a {@link Geometry} out of a "shp" {@link File}</li>
 * <li>Extraction of a {@link CoordinateReferenceSystem} out of a "shp" {@link File}</li>
 * <li>Extraction of a {@link CoordinateReferenceSystem} out of a {@link Geometry}</li>
 * <li>Transformation respectively mirroring of a {@link List} of {@link Geometry}s</li>
 *
 * <p>
 *
 * @author hahmann/meinert
 */
public class DataTools
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(DataTools.class);

    /**
     * Parses a {@link List} of {@link Geometry}s from {@code file}. {@link File} {@code file} is
     * assumed to be in CSV or Shape Format for later processing.
     *
     * @param file the CSV/Shape {@link File} object to be parsed
     *
     * @return {@link List} of {@link Geometry} objects
     */
    public static List<Geometry> loadGeometriesFromFile(File file)
    {
        List<Geometry> geometries = null;

        try
        {
            if (FilenameUtils.getExtension(file.getName()).equalsIgnoreCase("csv"))
            {
                geometries = loadGeometriesFromCsvFile(file);
            }
            else if (FilenameUtils.getExtension(file.getName()).equalsIgnoreCase("shp"))
            {
                geometries = loadGeometriesFromShapeFile(file);
            }
            else
            {
                logger.error("Unsupported Filetype: " + FilenameUtils.getExtension(file.getName()));
            }
        }
        catch (NullPointerException e)
        {
            logger.error("Given file is null.", e);
        }

        ArrayList<Geometry> transformedGeometries = transformGeometries(geometries);

        return transformedGeometries;
    }

    /**
     * Parses a {@link List} of {@link Geometry}s from {@code file}. {@link File} {@code file} is
     * assumed to be in CSV format (separator = ';') and to contain a column with header 'WKT'.
     *
     * @param file the CSV {@link File} object to be parsed
     *
     * @return {@link List} of {@link Geometry} objects
     */
    public static List<Geometry> loadGeometriesFromCsvFile(File file)
    {
        ArrayList<Geometry> loadedGeometries = new ArrayList<>();
        WKTReader reader = new WKTReader();
        CsvReader csvReader = null;

        try (FileReader fileReader = new FileReader(file))
        {
            csvReader = new CsvReader(fileReader, ';');
            csvReader.readHeaders();
            logger.trace("loadGeometriesFromCsvFile(), " + file);

            while (csvReader.readRecord())
            {
                try
                {
                    loadedGeometries.add(reader.read(csvReader.get("WKT")));
                }
                catch (ParseException ex)
                {
                    logger.error("loadGeometriesFromCsvFile(), ", ex);
                }
            }
            csvReader.close();
        }
        catch (IOException e)
        {
            logger.debug("loadGeometriesFromCsvFile(), " + "no file found", e);
        }
        finally
        {
            if (csvReader != null)
            {
                csvReader.close();
            }
        }

        return loadedGeometries;
    }

    /**
     * Parses a {@link List} of {@link Geometry}s from {@code file}. {@link File} {@code file} is
     * assumed to be in ESRI's Shape format.
     *
     * @param file the Shape {@link File} object to be parsed
     *
     * @return {@link List} of {@link Geometry} objects
     */
    public static List<Geometry> loadGeometriesFromShapeFile(File file)
    {
        ArrayList<Geometry> loadedGeometries = null;
        try
        {
            loadedGeometries = new ArrayList<>();
            Map<String, Object> map = new HashMap<>();
            map.put("url", file.toURI().toURL());
            DataStore dataStore = DataStoreFinder.getDataStore(map);
            String typeName = dataStore.getTypeNames()[0];

            FeatureSource<SimpleFeatureType, SimpleFeature> source = dataStore
                .getFeatureSource(typeName);
            // int srid = CRS.lookupEpsgCode(source.getSchema().getCoordinateReferenceSystem(),
            // false);
            FeatureCollection<SimpleFeatureType, SimpleFeature> collection = source.getFeatures();

            try (FeatureIterator<SimpleFeature> features = collection.features())
            {
                while (features.hasNext())
                {
                    SimpleFeature feature = features.next();
                    Object object = feature.getAttribute("the_geom");
                    Geometry geometry = (Geometry) object;
                    loadedGeometries.add(geometry);
                }
            }
        }
        catch (Exception e)
        {
            logger.debug("loadGeometriesFromShapeFile(), Exception=", e);
        }

        return loadedGeometries;
    }

    /**
     * Loading {@link Graph} data from network File.
     *
     * @param file network File
     * @return {@link Graph} object
     */
    public static Graph loadGraphFromShapeFile(File file)
    {

        Graph graph = null;
        try
        {

            Map<String, Object> map = new HashMap<>();
            map.put("url", file.toURI().toURL());
            DataStore dataStore = DataStoreFinder.getDataStore(map);
            String typeName = dataStore.getTypeNames()[0];

            FeatureSource<SimpleFeatureType, SimpleFeature> source = dataStore
                .getFeatureSource(typeName);
            // int srid = CRS.lookupEpsgCode(source.getSchema().getCoordinateReferenceSystem(),
            // false);
            FeatureCollection<SimpleFeatureType, SimpleFeature> fCollection = source.getFeatures();

            LineStringGraphGenerator lineStringGen = new LineStringGraphGenerator();
            FeatureGraphGenerator featureGen = new FeatureGraphGenerator(lineStringGen);

            try (FeatureIterator<SimpleFeature> features = fCollection.features())
            {
                while (features.hasNext())
                {
                    SimpleFeature feature = features.next();
                    featureGen.add(feature);
                }
            }
            graph = featureGen.getGraph();

        }
        catch (Exception e)
        {
            logger.debug("loadGraphFromShapeFile(), Exception=", e);
        }

        return graph;
    }

    /**
     * Loading LineString data from network File.
     *
     * @param file network File
     * @return FeatureCollection of .shp File
     */
    public static FeatureCollection<SimpleFeatureType, SimpleFeature> loadFeaturesFromShapeFile(
        File file)
    {

        FeatureCollection<SimpleFeatureType, SimpleFeature> fCollection = null;

        try
        {

            Map<String, Object> map = new HashMap<>();
            map.put("url", file.toURI().toURL());
            DataStore dataStore = DataStoreFinder.getDataStore(map);
            String typeName = dataStore.getTypeNames()[0];

            FeatureSource<SimpleFeatureType, SimpleFeature> source = dataStore
                .getFeatureSource(typeName);
            // int srid = CRS.lookupEpsgCode(source.getSchema().getCoordinateReferenceSystem(),
            // false);
            fCollection = source.getFeatures();

            return fCollection;

        }
        catch (Exception e)
        {
            // Logger.debug("loadGraphFromShapeFile(), Exception=", e);
        }

        return fCollection;

    }

    /**
     * Extracts the {@link CoordinateReferenceSystem} from a {@link File}.
     *
     * @param file the Shape or CSV {@link File} object to be parsed
     *
     * @return the {@link CoordinateReferenceSystem} of the {@link File}
     */
    public static CoordinateReferenceSystem getCRSFromFile(File file)
    {
        if (FilenameUtils.getExtension(file.getName()).equalsIgnoreCase("shp"))
        {
            try
            {
                Map<String, Object> map = new HashMap<>();
                map.put("url", file.toURI().toURL());
                DataStore dataStore = DataStoreFinder.getDataStore(map);
                String typeName = dataStore.getTypeNames()[0];

                FeatureSource<SimpleFeatureType, SimpleFeature> source = dataStore
                    .getFeatureSource(typeName);
                return source.getSchema().getCoordinateReferenceSystem();
            }
            catch (Exception e)
            {
                logger.debug("loadGeometriesFromShapeFile(), Exception=", e);
            }
        }

        return null;
    }

    // TODO at the moment there is no opportunity to decode a CoordinateReferenceSystem from
    // a WKT (see the following link in case of examples how to determine CRS in WKT -
    // http://www.gdal.org/gdalsrsinfo.html

    /**
     * Decodes the {@link CoordinateReferenceSystem} from the a SRID obtained from {@code geometry}
     *
     * @param geometry the geometry object to get the {@link CoordinateReferenceSystem} from
     *
     * @return the {@link CoordinateReferenceSystem} from the a SRID obtained from {@code geometry}
     *         or {@code null} if not matching {@link CoordinateReferenceSystem} has been found
     */
    public static CoordinateReferenceSystem getCRSFromGeometry(Geometry geometry)
    {
        int srid = geometry.getSRID();
        CoordinateReferenceSystem crs = null;
        try
        {
            crs = CRS.decode("EPSG:" + srid);
        }
        catch (FactoryException e)
        {
            logger.error("DataTools.getCRSFromGeometry(), unknown srid=" + srid, e);
        }

        return crs;
    }

    /**
     * Transforms the imported {@code boundaries} into the right alignment (from the view of java).
     * <p>
     * In fact that means that the imported {@link List} of {@link Geometry}s is mirrored. The
     * reason for that is that the y-axe in java shows to the bottom, instead to the top.
     *
     * @param geometries object is a {@link ArrayList} of {@link Geometry} of the boundaries
     *
     * @return a {@link ArrayList} of {@link Geometry} of the {@code boundaries} which were now
     *         transformed.
     */
    public static ArrayList<Geometry> transformGeometries(List<Geometry> geometries)
    {
        if (geometries == null)
            return null;
        MathTransform transform = new org.geotools.referencing.operation.transform.AffineTransform2D(
            1, 0, 0, -1, 0, 0);
        ArrayList<Geometry> transformedGeometries = new ArrayList<>();
        for (Geometry geometry : geometries)
        {
            try
            {
                transformedGeometries.add(JTS.transform(geometry, transform));
            }
            catch (MismatchedDimensionException | TransformException e)
            {
                logger.error("transformGeometries(), error when transforming data", e);
            }
        }

        return transformedGeometries;
    }

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

    /**
     * Parses wayPoint-Strings to Objects
     *
     * @param wayPoints String representation of wayPoints
     * @return {@link Geometry}-List of points
     */
    public static ArrayList<Geometry> loadWayPointsFromString(String wayPoints)
    {
        ArrayList<Geometry> geometriesReal = new ArrayList<>();
        ArrayList<Geometry> geometriesJava = new ArrayList<>();
        WKTReader readerWKT = new WKTReader();

        String wkt = wayPoints.substring((wayPoints.indexOf("[") + 1), wayPoints.length());

        String[] wkts = wkt.split(",");

        for (String geometry : wkts)
        {
            try
            {
                geometriesReal.add(readerWKT.read(geometry));
            }
            catch (ParseException ex)
            {
                logger.error("loadGeometriesFromCsvFile(), ", ex);
            }
        }
        geometriesJava = transformGeometries(geometriesReal);

        return geometriesJava;
    }

    /**
     * Central method to parse csv File and initialize {@link PedestrianAgent} objects.
     *
     * @param file csv-File with pedestrians mission
     * @return List of {@link PedestrianAgent}s
     */
    public static List<PedestrianAgent> loadDataFromCsvFile(File file)
    {
        ArrayList<PedestrianAgent> loadedPedestrians = new ArrayList<>();
        WKTReader readerWKT = new WKTReader();
        // CsvReader readerStr = new CsvReader(null);
        CsvReader csvReader = null;

        try (FileReader fileReader = new FileReader(file))
        {
            csvReader = new CsvReader(fileReader, ',');
            csvReader.readHeaders();
            logger.trace("loadDataFromCsvFile(), " + file);

            int index = 0;

            while (csvReader.readRecord())
            {
                try
                {
                    Geometry startPositionReal = readerWKT.read(csvReader.get("startWKT"));
                    Geometry startPositionJava = transformGeometry(startPositionReal);

                    // Parsing Startkoordinate
                    Coordinate startPosition = startPositionJava.getGeometryN(0).getCoordinate();
                    double x = startPosition.x;
                    double y = startPosition.y;

                    String wayfindModel = csvReader.get("mentalModel");

                    String wayPoints = csvReader.get("wktWayPoints");

                    List<Geometry> waypoints = loadWayPointsFromString(wayPoints);
                    // waypoints = loadWayPointsFromString(wayPoints);

                    // System.out.println(index + ": " + x + " " + y + " WP: " + waypoints);

                    loadedPedestrians
                        .add(new PedestrianAgent(index, x, y, waypoints, wayfindModel));
                    index++ ;

                }
                catch (ParseException ex)
                {
                    logger.error("loadGeometriesFromCsvFile(), ", ex);
                }
            }
            csvReader.close();
        }
        catch (IOException e)
        {
            logger.debug("loadDataFromCsvFile(), " + "no file found", e);
        }
        finally
        {
            if (csvReader != null)
            {
                csvReader.close();
            }
        }

        return loadedPedestrians;
    }

}
