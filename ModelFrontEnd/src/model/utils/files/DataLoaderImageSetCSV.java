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

public class DataLoaderImageSetCSV extends AbstractDataLoader{

    public static int INVALID_LABEL = Integer.MAX_VALUE;
    
    protected static String str_FILENAME_METADATA = "metaData.csv";
    protected static String str_FILENAME_VALUES = "values.csv";
    protected static String str_FILENAME_LABELS = "labels.csv";
    
    protected static int MODE_BINARY  = 0;
    protected static int MODE_0To1    = 1;
    protected static int MODE_255     = 255;
    
    protected int m_nMode;
    protected int m_nofCannels;
    
    protected double [] m_arrValues;
    protected int [] m_arrLabels;

    protected int loadMetaData(String par_strFilename){
        
        File file = new File(par_strFilename);
        try (Scanner scanner = new Scanner(file)){
        
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
            m_nMode = Integer.parseInt(arrLineElements[1]);

            // h
            strLine = scanner.next();
            arrLineElements = strLine.split(",");
            m_nofCannels = Integer.parseInt(arrLineElements[1]);
            
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
    
    protected int loadValues(String par_strFilename){
        
        int nofDims = m_arrSampleDims.length;
        int nofElementsPerSample = 1;
        for(int i=0; i<nofDims; i++){
            
            nofElementsPerSample *= m_arrSampleDims[i];
        }
        nofElementsPerSample *= m_nofCannels;
        
        m_arrValues = new double[ m_nofSamples*nofElementsPerSample ];
        File file   = new File(par_strFilename);
        
        try(Scanner scanner = new Scanner(file)){
            
            scanner.useDelimiter("\n");
            String strLine;
            String [] arrLineElements;
            double [] arrSample = new double [ nofElementsPerSample ]; 
            double value;
            
            int i=0;
            for(int si=0; si<m_nofSamples && scanner.hasNext(); si++){

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
        File file   = new File(par_strFilename);
        
        try(Scanner scanner = new Scanner(file)){
            
            scanner.useDelimiter("\n");
            String strLine;

            int i=0;
            while(scanner.hasNext()){
                
                strLine = scanner.next();
                strLine = strLine.replaceAll("\r", "");
                int label = Integer.parseInt(strLine);
                if(label == INVALID_LABEL){
                    
                    String message = "Invalid label value " + Integer.toString(label) + "at i=" + Integer.toString(i);
                    Exception e = new Exception(message);
                    throw e;
                }
                else
                    m_arrLabels[ i++ ] = label;
            }
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
        
        loadValues(m_strDataPath + str_FILENAME_VALUES);
        loadLabels(m_strDataPath + str_FILENAME_LABELS);
    }
    
    @Override
    public void init(){

        File dirDataPath;
        dirDataPath = new File(m_strDataPath);
        if(dirDataPath.isDirectory()){
            
            loadMetaData(m_strDataPath + str_FILENAME_METADATA);
        }
    }
    
    @Override
    public int setParams(String par_strDataPath){
        
        m_strDataPath = par_strDataPath;
        if(!m_strDataPath.endsWith(File.separator))
            m_strDataPath += File.separator;
        File dataPath;
        dataPath = new File(m_strDataPath);
        return (dataPath.exists())? 0 : 1;
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
    
    @Override
    public int getLabel(int par_index){
        
        int label = INVALID_LABEL;
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
                
                // first entry
                arrLabelNames[ nofClassesFound++ ] = labelTemp;
            }
            else if(nofClassesFound<m_nofClasses){
                
                int backTracki = nofClassesFound-1;
                boolean alreadyListed = false;
                while(backTracki>=0 && !alreadyListed){
                    
                    int listedLabel = arrLabelNames[backTracki];
                    alreadyListed = labelTemp == listedLabel;
                    backTracki--;
                }
                if(!alreadyListed){
                    
                    // add as new listing
                    arrLabelNames[ nofClassesFound++ ] = labelTemp;
                }
            }
        }
        if(nofClassesFound < m_nofClasses){
            System.err.print("Not all classes represented in subset.");
        }
        return arrLabelNames;
    }
    
    public DataLoaderImageSetCSV(){
        
    }
    
}
