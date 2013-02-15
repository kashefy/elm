/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

/**
 *
 * @author woodstock
 */
import java.io.*;
import java.util.Scanner;

public class DataLoaderWeightSet extends DataLoaderImageSetCSV {
        
    protected String strSuffixValuesWeights = "weights.csv";
    
    @Override
    protected int loadMetaData(String par_strFilename){
        
        File file = new File(par_strFilename);
        try{
        
            Scanner scanner = new Scanner(file);
            scanner.useDelimiter("\n");
            String strLine;
            String [] arrLineElements;
            
            // n
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
                        
            m_arrSampleDims = new int[2];
            m_arrSampleDims[ FileIO.DIM_INDEX_ROWS ] = 1;
            m_arrSampleDims[ FileIO.DIM_INDEX_COLS ] = arrLineElements.length;
            
            m_nofSamples++;
            while(scanner.hasNext()){
                
                strLine = scanner.next();
                if(strLine.length()>0)
                    m_nofSamples++;
            }
            
            m_nofCannels = 1;
            
            scanner.close();
            
        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
            return 1;
        }
        return 0;
    }
    
    @Override
    protected int loadValues(String par_strFilename){
        
        int nofDims = m_arrSampleDims.length;
        int nofElementsPerSample = 1;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        nofElementsPerSample *= m_nofCannels;
        
        m_arrValues = new double[ m_nofSamples*nofElementsPerSample ];
        File file   = new File(par_strFilename);
        
        try{
            
            Scanner scanner = new Scanner(file);
            scanner.useDelimiter("\n");
            String strLine;
            String [] arrLineElements;
            double [] arrSample = new double [ nofElementsPerSample ]; 
            double value;
            
            int i=0;
            for(int si=0; scanner.hasNext(); si++){

                strLine = scanner.next();
                strLine = strLine.replaceAll("\r", "");
                arrLineElements = strLine.split(",");
                for(int eli=0; eli<nofElementsPerSample; eli++){
            
                    value = Double.parseDouble(arrLineElements[ eli ]);
                    arrSample[ eli ] = value;
                    m_arrValues[ i++ ] = value;
                }
            }
            
            scanner.close();

        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
            return 1;
        }
        
        return 0;
    }
    
    protected int loadLabels(String par_strFilename){
                
        m_arrLabels = new int[ m_nofSamples ];
        for(int i=0; i<m_nofSamples; i++)
            m_arrLabels[i]=i;
        return 0;
    }
    
    @Override
    public void init(){

        loadMetaData(new File(m_strDataPath, strSuffixValuesWeights).getPath());
    }
    
    @Override
    public void load(){
        
        loadValues(new File(m_strDataPath, strSuffixValuesWeights).getPath());
        loadLabels(null);
    }
    
    @Override
    public int setParams(String par_strDataPath){
        
        m_strDataPath = par_strDataPath;
        File dataPath;
        dataPath = new File(m_strDataPath);
        return (dataPath.exists())? 0 : 1;
    }
    
    public void setWeightValueFilename(String par_strFilename){
        
        strSuffixValuesWeights = par_strFilename;
    }
        
    @Override
    public double[] getSample(int par_index){
        
        double [] sampleToExport = null;
        int nofElementsPerSample = 1;
        int nofDims = m_arrSampleDims.length;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        nofElementsPerSample *= m_nofCannels;
        
        if(par_index < m_nofSamples){
            
            sampleToExport = new double[ nofElementsPerSample ]; 
            System.arraycopy(m_arrValues, nofElementsPerSample*par_index, sampleToExport, 0, nofElementsPerSample);
        }
                
        return sampleToExport;
    }
    
}
