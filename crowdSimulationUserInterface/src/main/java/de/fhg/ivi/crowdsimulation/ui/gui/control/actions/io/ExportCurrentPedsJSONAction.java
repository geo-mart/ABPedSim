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

public class ExportCurrentPedsJSONAction extends AbstractAction
{
    /**
    *
    */
    private static final long   serialVersionUID = 1L;

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory
        .getLogger(ExportCurrentPedsJSONAction.class);

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Export Point-JSON...";

    public ExportCurrentPedsJSONAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.NEUTER, blackColor);
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
            exportCurrentPedsJSON(crowdSimulator);
        }
        catch (IOException e1)
        {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }

    }

    /**
     * Exports current position in GEOJSON File named pedestrian_point{date}.json.
     *
     * @param crowdSimulator
     * @throws IOException
     */
    public void exportCurrentPedsJSON(CrowdSimulator crowdSimulator) throws IOException
    {
        FileWriter fw;

        String fileName = new SimpleDateFormat("'pedestrian_point'yyyyMMddHHmm'.json'")
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
                    bw.write(pede.currentPosition2JSON());
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

    /**
     * Builds GEOJSON-String of current positions.
     *
     * @param crowdSimulator
     * @return GEOJSON-String of current positions
     */
    public String exportCurrentPedsString(CrowdSimulator crowdSimulator)
    {

        String newline = System.getProperty("line.separator");

        StringBuilder json = new StringBuilder();

        json.append(
            "{ \\\"type\\\": \\\"FeatureCollection\\\",\\r\\n\" + \"\\t\\\"features\\\": [");
        json.append(newline);

        json.append("{ \"type\": \"FeatureCollection\",\r\n" + "\t\"features\": [");
        json.append(newline);

        String start = "";
        for (Crowd cro : crowdSimulator.getCrowds())
        {
            for (Pedestrian pede : cro.getPedestrians())
            {
                json.append(start);
                json.append(newline);
                json.append(pede.currentPosition2JSON());
                start = ",";

            }
        }

        json.append(newline);
        json.append("    ]\r\n}");

        return json.toString();

    }

}
