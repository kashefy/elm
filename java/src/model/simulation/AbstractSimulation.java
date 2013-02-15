/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.simulation;

/**
 *
 * @author woodstock
 */
import model.utils.files.*;

public abstract class AbstractSimulation {
    
    protected SimulationParams m_params;
        
    // stimuli
    protected AbstractDataLoader m_dataLoader;
    
    public abstract void setParams(SimulationParams params);
    
    public abstract void init();
    
    public abstract void run();
    
    protected int[] determineLabelSet(int starti, int endi){
        
        // determine which label a ZNeuron is responding to
        int nofCauses = m_params.getNofCauses();
        
        int [] arrLabelNames = new int [ nofCauses ];
        int nofSamples = endi-starti+1;
        for(int ci=0; ci<nofCauses; ci++)
            arrLabelNames[ ci ] = -1;
        
        int nofCausesFound = 0;
        for(int li=0; li<nofSamples; li++){
            
            int labelTemp = m_dataLoader.getLabel(li);
            
            if(nofCausesFound==0){
                
                // first entry
                arrLabelNames[ nofCausesFound++ ] = labelTemp;
            }
            else if(nofCausesFound<nofCauses){
                
                int backTracki = nofCausesFound-1;
                boolean alreadyListed = false;
                while(backTracki>=0 && !alreadyListed){
                    
                    int listedLabel = arrLabelNames[backTracki];
                    alreadyListed = labelTemp == listedLabel;
                    backTracki--;
                }
                if(!alreadyListed){
                    
                    // add as new listing
                    arrLabelNames[ nofCausesFound++ ] = labelTemp;
                }
            }
        }
        if(nofCausesFound < nofCauses){
            System.err.print("Not all classes represented in subset.");
        }
        return arrLabelNames;
    }
}
