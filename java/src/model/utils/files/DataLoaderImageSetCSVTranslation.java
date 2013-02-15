/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

import java.util.Random;
import java.io.*;
import java.util.Scanner;

/**
 *
 * @author woodstock
 */
public class DataLoaderImageSetCSVTranslation extends DataLoaderImageSetCSV{
    
    int [][] m_arrLocation;
    public static int SCENE_ROWS = 28+28;
    public static int SCENE_COLS = 28+28;
    
    protected int m_nofElementsPerScene;
    
    Random m_backgroundNoiseGenerator;
    
    @Override
    public void init(){

        super.init();
        m_arrLocation = new int [ m_nofSamples ][ 2 ];
        
        m_backgroundNoiseGenerator = new Random();
        
        m_nofElementsPerScene = SCENE_ROWS * SCENE_COLS;
    }
    
    @Override
    public void load(){
        
        super.load();
        
        // create locations
        int nofRowsInSample = m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ];
        int nofColsInSample = m_arrSampleDims[ FileIO.DIM_INDEX_COLS ];
        int nofRowsInScene = SCENE_ROWS;
        int nofColsInScene = SCENE_COLS;
        
        int rangeForRows = nofRowsInScene - nofRowsInSample + 1; // [0,n)
        int rangeForCols = nofColsInScene - nofColsInSample + 1; // [0,n)
        
        Random generator = new Random();
        
        for(int i=0; i<m_nofSamples; i++){
            
            int placementRow = 0;//generator.nextInt(rangeForRows);
            int placementCol = 0;//generator.nextInt(rangeForCols);
            
            m_arrLocation[i][ FileIO.DIM_INDEX_ROWS ] = placementRow;
            m_arrLocation[i][ FileIO.DIM_INDEX_COLS ] = placementCol;
        }
    }
    
    public double[] loadNoiseFromFile(String par_strFilename){
        
        int nofElementsPerSample = SCENE_ROWS*SCENE_COLS; 
        double [] noiseFromFile = new double [ nofElementsPerSample ];
        
        File file   = new File(par_strFilename);
        
        try{
            
            Scanner scanner = new Scanner(file);
            scanner.useDelimiter("\n");
            String strLine;
            String [] arrLineElements;
            double [] arrSample = new double [ nofElementsPerSample ]; 
            double value;
            
            int i=0;
            while (scanner.hasNext()){

                strLine = scanner.next();
                strLine = strLine.replaceAll("\r", "");
                arrLineElements = strLine.split(",");
                for(int eli=0; eli<nofElementsPerSample; eli++){
            
                    value = Double.parseDouble(arrLineElements[ eli ]);
                    arrSample[ eli ] = value;
                }
            }
            scanner.close();
            noiseFromFile = arrSample;

        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
        }
        
        return noiseFromFile;
    }
    
    public int[] getLocation(int par_index){

        int [] locToExport = new int[2];
        System.arraycopy(m_arrLocation[par_index], 0, locToExport, 0, 2);
        return locToExport;
    }
    
    @Override
    public int [] getDims(){
        
        int [] dimsToExport = new int[]{SCENE_ROWS, SCENE_COLS};
        return dimsToExport;
    }
    
    @Override
    public double[] getSample(int par_index){
        
        int nofElementsPerSample = 1;
        int nofDims = m_arrSampleDims.length;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        nofElementsPerSample *= m_nofCannels;
        int nofRowsInSample = m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ];
        int nofColsInSample = m_arrSampleDims[ FileIO.DIM_INDEX_COLS ];
        
        double [] scene = new double[ m_nofElementsPerScene ];
        double [] noise = this.loadNoiseFromFile(m_strDataPath + "n.csv");
        
        // add noise to scene background by flipping values
        for(int i=0; i<m_nofElementsPerScene; i++){
               
            scene[i] = noise[i];
//            double r = m_backgroundNoiseGenerator.nextGaussian(); 
//            if(Math.abs(r)>1.7)
//
//                scene[i] = 1 - scene[i];
            
        }
        
        if(par_index < m_nofSamples){
            
            int sampleOffset = nofElementsPerSample*par_index;
            int transRow = m_arrLocation[ par_index ][ FileIO.DIM_INDEX_ROWS ];
            int transCol = m_arrLocation[ par_index ][ FileIO.DIM_INDEX_COLS ];
            for(int r=0; r<nofRowsInSample; r++){
                
                int rowOffsetInSample = sampleOffset + (r*nofColsInSample);
                int rowInScene = r + transRow;
                int rowOffsetInScene = rowInScene * SCENE_COLS;
                for(int c=0; c<nofColsInSample; c++){
                    
                    int colInScene = c + transCol;
                    //if(scene[ rowOffsetInScene + colInScene ] != 1) // upper bound
                    //    scene[ rowOffsetInScene + colInScene ] += m_arrValues[ rowOffsetInSample + c ];
                    scene[ rowOffsetInScene + colInScene ] = m_arrValues[ rowOffsetInSample + c ];
                }
            }
        }
                
        return scene;
    }
    
    public DataLoaderImageSetCSVTranslation(){
        
    }
}
