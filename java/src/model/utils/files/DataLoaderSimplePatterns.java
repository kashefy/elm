/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

/**
 *
 * @author woodstock
 */
import java.util.Random;
import model.simulation.SimulationParams;
import model.features.OrientationMapTest;

import java.io.*;
import java.util.Scanner;

public class DataLoaderSimplePatterns extends AbstractDataLoader{
    
    public static int INVALID_LABEL = Integer.MAX_VALUE;
    
    protected static String strFILENAME_METADATA = "metaData.csv";
    
    SimulationParams m_simParams;
    double [][] m_barSet;
    
    protected double [] m_arrValues;
    protected int [] m_arrLabels;
    
    protected double m_nNoiseThreshold;
      
    @Override
    public int setParams(String par_strDataDir){
        
        m_strDataPath = par_strDataDir;
     
        return 0;
    }

    @Override
    public void init(){
        
        File dirDataPath;
        dirDataPath = new File(m_strDataPath);
        if(dirDataPath.isDirectory()){
            
            loadMetaData(new File(dirDataPath, strFILENAME_METADATA));
        }

        int nofRows = m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = m_arrSampleDims[ FileIO.DIM_INDEX_COLS ];
        int nofStimElements = nofRows * nofCols;
        
        m_barSet = new double [ m_nofClasses ][ nofStimElements ];
        int nofOrientations = m_nofClasses;
        double angleIncInDeg = 180.0/(double)nofOrientations;
        double angleInDeg = 0;
        Random angleGenerator = new Random();
        
        for(int i=0; i<nofOrientations; i++){
            
            // select angle from uniform distribution
            angleInDeg = angleGenerator.nextInt(180);
            
            m_barSet[i] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,3,angleInDeg);

            angleInDeg += angleIncInDeg;
        }
        
        m_nNoiseThreshold = 2.3;
        
//        m_barSet[0] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,90);
//        m_barSet[1] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,0);
//        m_barSet[2] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,45);
//        m_barSet[3] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,60);
//        m_barSet[4] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,120);
//        m_barSet[5] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,150);
    }
    
    protected int loadMetaData(File par_file){
        
        try(Scanner scanner = new Scanner(par_file)){
            scanner.useDelimiter("\n");
            String strLine;
            String [] arrLineElements;
            
            // n
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            m_nofSamples = Integer.parseInt(arrLineElements[1]);
            
            m_arrSampleDims = new int[2];
            
            // r
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ] = Integer.parseInt(arrLineElements[1]);

            // c
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            m_arrSampleDims[ FileIO.DIM_INDEX_COLS ] = Integer.parseInt(arrLineElements[1]);

            // m
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            //m_nMode = Integer.parseInt(arrLineElements[1]);

            // h
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            //m_nofCannels = Integer.parseInt(arrLineElements[1]);
            
            // l
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            m_nofClasses = Integer.parseInt(arrLineElements[1]);
            
            scanner.close();
            
        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
            return 1;
        }
        return 0;
    }

    @Override
    public void load(){

        int nofDims = m_arrSampleDims.length;
        int nofElementsPerSample = 1;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        
        m_arrValues = new double[ m_nofSamples*nofElementsPerSample ];
        m_arrLabels = new int[ m_nofSamples ];
        Random generator = new Random();
        Random noiseGenerator = new Random();
        
        for(int si=0; si<m_nofSamples; si++){
            
            int barSetIndex = generator.nextInt(m_nofClasses);
            double [] sample = new double[ nofElementsPerSample ];
            System.arraycopy(m_barSet[barSetIndex], 0, sample, 0, nofElementsPerSample);
            
            // add noise by flipping values
            for(int j=0; j<nofElementsPerSample; j++){
               
                double r = noiseGenerator.nextGaussian(); 
                if(Math.abs(r) > m_nNoiseThreshold)
                   
                    sample[j] = 1 - sample[j];
            }
            
            System.arraycopy(sample, 0, m_arrValues, nofElementsPerSample*si, nofElementsPerSample);
            m_arrLabels[si] = barSetIndex;
        }
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
            System.arraycopy(m_arrValues, nofElementsPerSample*par_index, sampleToExport, 0, nofElementsPerSample);
        }
                
        return sampleToExport;   
    }

    @Override
    public int getLabel(int par_index){
        
        int label = -1;
        
        if(par_index < m_nofSamples){
            
            label = m_arrLabels[par_index];
        }      
        return label;
    }
    
    public int[] determineLabelSet(int starti, int endi){
        
        int [] arrLabelNames = new int [ m_nofClasses ];
        int nofSamples = endi-starti+1;
        for(int ci=0; ci<m_nofClasses; ci++)
            arrLabelNames[ ci ] = INVALID_LABEL;
        
        int nofClassesFound = 0;
        for(int li=0; li<nofSamples; li++){
            
            int labelTemp = getLabel(li);
            
            if(nofClassesFound==0){
                
                arrLabelNames[ nofClassesFound++ ] = labelTemp; // first entry
            }
            else if(nofClassesFound < m_nofClasses){
                
                int backTracki = nofClassesFound-1;
                boolean alreadyListed = false;
                while(backTracki>=0 && !alreadyListed){
                    
                    int listedLabel = arrLabelNames[backTracki];
                    alreadyListed = labelTemp == listedLabel;
                    backTracki--;
                }
                if(!alreadyListed){
                    
                    arrLabelNames[ nofClassesFound++ ] = labelTemp; // add as new listing
                }
            }
        }
        if(nofClassesFound < m_nofClasses){
            System.err.print("Not all classes represented in subset.");
        }
        return arrLabelNames;
    }
    
    public DataLoaderSimplePatterns(){
        
    }
}
