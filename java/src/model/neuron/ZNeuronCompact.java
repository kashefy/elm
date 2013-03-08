/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.neuron;

/**
 *
 * @author woodstock
 */
public class ZNeuronCompact extends ZNeuron{
    
    
    private int [] m_spikeHistoryCompact;
      
    @Override
    protected void initHistory(){
        
        m_spikeHistoryCompact = new int[ m_nofEvidence ];
    }
    
    @Override
    protected void updateHistory(int [] par_spikes){

        for(int yi=0; yi<m_nofEvidence; ++yi){

            if(par_spikes[yi] == 1){
            
                m_spikeHistoryCompact[yi] = m_historyLength;
            }
        }
    }
    
    @Override
    protected void resetHistory(){
        
        m_spikeHistoryCompact = new int[ m_nofEvidence ];
    }
    
    @Override
    protected int hasInputFiredRecently(int par_index){
               
        int nInputFiredRecently;

        nInputFiredRecently = m_spikeHistoryCompact[ par_index ];

        return nInputFiredRecently;
    } 

    /*
    * padvance spike history by n time steps
    * @param  par_nSteps - number of tie stemps to advanceHistory in
    */
    @Override
    protected void advanceHistory(int par_nSteps){
        
        // move spike History in time by decrementing times by 1 towards 0, nothing < 0
        // perform for all nodes
        for(int yi=0; yi<m_nofEvidence; ++yi){
            
            int spikeHistoryVal = m_spikeHistoryCompact[yi];
            if(spikeHistoryVal > 0){
                
                m_spikeHistoryCompact[yi] = --spikeHistoryVal;
            }
        }
    }
    
}
