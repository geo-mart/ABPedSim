package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io;

import java.awt.event.ActionEvent;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.Action;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.objects.Crowd;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import jiconfont.icons.FontAwesome;

public class ExportJSONAction extends AbstractAction
{

    /**
     *
     */
    private static final long   serialVersionUID = 1L;

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory.getLogger(ExportJSONAction.class);

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Export LineString-JSON...";

    public ExportJSONAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.DOWNLOAD, blackColor);
        putValue(Action.SHORT_DESCRIPTION, name);
    }

    @Override
    public void actionPerformed(ActionEvent e)
    {
        CrowdSimulator crowdSimulator = crowdSimulation.getCrowdSimulator();
        // MapPanel mapPanel = crowdSimulation.getPanelManager().getPanel(MapPanel.class);

        crowdSimulator.pauseSimulation();

        try
        {
            exportJSON(crowdSimulator);
        }
        catch (IOException e1)
        {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }

    }

    /**
     * Exports GEOJSON of trajectory, first to console for CityScope Visualization, then to GEOJSON
     * file named pedestrian{date}.json
     *
     * @param crowdSimulator
     * @throws IOException
     */
    public void exportJSON(CrowdSimulator crowdSimulator) throws IOException
    {

        // in Konsole ausgeben

        StringBuilder json = new StringBuilder();

        json.append("L{\"type\":\"FeatureCollection\"," + "\"features\":[");

        String startSeperator = "";

        for (Crowd cro : crowdSimulator.getCrowds())
        {
            for (Pedestrian pede : cro.getPedestrians())
            {

                json.append(startSeperator);
                json.append(pede.trajectory2JSONLine());
                startSeperator = ",";

            }

        }

        json.append("]}");
        String result = json.toString();

        System.out.println(result);

        // in Datei schreiben
        FileWriter fw;

        String fileName = new SimpleDateFormat("'pedestrian'yyyyMMddHHmm'.json'")
            .format(new Date());

        fw = new FileWriter(fileName);

        BufferedWriter bw = new BufferedWriter(fw);
        try
        {
            bw.write("{ \"type\": \"FeatureCollection\",\r\n" + "\t\"features\": [");
            bw.newLine();

            String start = "";
            for (Crowd cro : crowdSimulator.getCrowds())
            {
                for (Pedestrian pede : cro.getPedestrians())
                {
                    bw.write(start);
                    bw.newLine();
                    bw.write(pede.trajectory2JSON());
                    start = ",";

                }
            }

            bw.newLine();
            bw.write("    ]\r\n}");

            bw.close();
        }
        finally
        {
            // Wird trotzdem nach dem return ausgeführt!
            if (bw != null)
            {
                bw.close();
            }
        }

    }

}
