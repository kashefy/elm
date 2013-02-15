/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

import java.awt.Image;

/**
 *
 * @author woodstock
 */
import java.io.*;
import java.util.Random;

public class DataLoaderNoise extends DataLoaderSimplePatterns{

    Random m_generator;
    
    @Override
    public void init(){

        File dirDataPath;
        dirDataPath = new File(m_strDataPath);
        if(dirDataPath.isDirectory()){
            
            super.loadMetaData(new File(dirDataPath, strFILENAME_METADATA));
        }
        
        int nofRows = m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = m_arrSampleDims[ FileIO.DIM_INDEX_COLS ];
        int nofStimElements = nofRows * nofCols;
        
        m_generator = new Random();
        m_nNoiseThreshold = 2.3;
    }

    @Override
    public void load(){
        
    }

    @Override
    public double [] getSample(int par_index){
        
        double [] sampleToExport = null;
        int nofElementsPerSample = 1;
        int nofDims = m_arrSampleDims.length;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        
        if(par_index < m_nofSamples){
            
            sampleToExport = new double[ nofElementsPerSample ]; 
            // add noise by flipping values
            for(int i=0; i<nofElementsPerSample; i++){
               
                double r = m_generator.nextGaussian(); 
                if(Math.abs(r) > m_nNoiseThreshold)
                   
                    sampleToExport[i] = 1 - sampleToExport[i];
            }
        }
                
        return sampleToExport;  
    } 

    @Override
    public int getLabel(int par_index){
        
        if(par_index < m_nofSamples){
            
            return par_index;
        }
        else return -1;
    }
}
