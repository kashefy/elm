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
import java.util.*;

public class DataLoaderImageSetCSV_incremental extends DataLoaderImageSetCSV{

    protected int m_cursor;
    protected int m_nofElementsPerSample;
    protected File m_data_dir;

    protected int loadMetaData(File par_file){
        
        try (Scanner scanner = new Scanner(par_file)){
        
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
    
    /**
     * @brief load next values
     * @param file
     * @return 
     */
    protected int loadValues(File par_file){
        
        if(m_cursor < m_nofSamples){
            
            m_arrValues = new double[ m_nofElementsPerSample ];

            try(Scanner scanner = new Scanner(par_file)){

                scanner.useDelimiter("\n");
                String strLine = "";
                String [] arrLineElements;
                double [] arrSample = new double [ m_nofElementsPerSample ]; 
                double value;

                for(int si=0; si<=m_cursor && scanner.hasNext(); si++){

                    strLine = scanner.next();
                }
                strLine = strLine.replaceAll("\r", "");
                arrLineElements = strLine.split(",");
                for(int eli=0; eli<m_nofElementsPerSample; eli++){

                    value = Double.parseDouble(arrLineElements[ eli ]);
                    arrSample[ eli ] = value;
                }
                m_arrValues = arrSample;
                scanner.close();
            }
            catch(Exception e){

                System.err.println("Error: " + e.getMessage());
                return 1;
            }
            return 0;
        }
        return 1;
    }

    /**
     * @brief load next labels
     * @param file
     * @return 
     */
    protected int loadLabels(File par_file){
                
        if(m_cursor < m_nofSamples){
            
            m_arrLabels = new int[ 1 ];
            try(Scanner scanner = new Scanner(par_file)){

                scanner.useDelimiter("\n");
                String strLine = "";

                int i=0;
                while(scanner.hasNext() && i<=m_cursor){

                    strLine = scanner.next();
                    i++;
                }
                strLine = strLine.replaceAll("\r", "");
                int label = Integer.parseInt(strLine);
                if(label == INVALID_LABEL){
                    
                    String message = "Invalid label value " + Integer.toString(label) + "at i=" + Integer.toString(i);
                    Exception e = new Exception(message);
                    throw e;
                }
                else{
                    m_arrLabels[0] = label;
                }
                scanner.close();
            }
            catch(Exception e){

                System.err.println("Error: " + e.getMessage());
                return 1;
            }        
            return 0;
        }
        else{
            return 1;
        }
    }
    
    @Override
    /**
     * load next
     */
    public void load(){
        
        m_cursor += 1;
        loadValues(new File(m_data_dir, str_FILENAME_VALUES));
        loadLabels(new File(m_data_dir, str_FILENAME_LABELS));
    }
    
    @Override
    public void init(){

        m_data_dir = new File(m_strDataPath);
        if(m_data_dir.isDirectory()){
            
            loadMetaData(new File(m_data_dir, str_FILENAME_METADATA));
        }
        m_cursor = -1;
        int nofDims = m_arrSampleDims.length;
        m_nofElementsPerSample = 1;
        for(int i=0; i<nofDims; i++){

            m_nofElementsPerSample *= m_arrSampleDims[i];
        }
        m_nofElementsPerSample *= m_nofCannels;    
    }
    
    @Override
    public int setParams(String par_strDataPath){
        
        m_strDataPath = par_strDataPath;
        //if(!m_strDataPath.endsWith(File.separator))
        //    m_strDataPath += File.separator;
        File dataPath;
        dataPath = new File(m_strDataPath);
        return (dataPath.exists())? 0 : 1;
    }
        
    @Override
    public double[] getSample(int par_index){
        
        double [] sampleToExport = null;
        if(par_index == m_cursor && par_index < m_nofSamples){

            sampleToExport = new double[ m_nofElementsPerSample ]; 
            System.arraycopy(m_arrValues, 0, sampleToExport, 0, m_nofElementsPerSample);
        }
        return sampleToExport;
    }
    
    @Override
    public int getLabel(int par_index){
        
        int label = INVALID_LABEL;
        if(par_index == m_cursor){
            
            label = m_arrLabels[0];
        }      
        return label;
    }
    
    /**
     * determine label set contained within range
     * @param start index
     * @param end index
     * @return set of label names
     */
    public int[] determineLabelSet(int starti, int endi){
        
        int position_old = m_cursor;
        Set<Integer> label_names_set = new TreeSet<>();

        m_cursor = starti-1;
        int li=starti;
        while(li<endi && label_names_set.size()<m_nofClasses){
            
            load();
            int label = getLabel(li);
            
            if(label == INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(label) + "at i=" + Integer.toString(li);
                System.err.println(message);
            }
            else{
                label_names_set.add(label);
            }
            li++;
        }
        if(label_names_set.size() < m_nofClasses){
            System.err.print("Not all classes represented in subset.");
        }
        m_cursor = position_old;
        
        int [] arr_label_names = new int [ m_nofClasses ];
        Arrays.fill(arr_label_names, INVALID_LABEL);
        int i = 0;
        for( Integer name : label_names_set ) {
          arr_label_names[i++] = name; //note the autounboxing here
        }
        
        return arr_label_names;
    }
    
    public DataLoaderImageSetCSV_incremental(){
        
    }
    
    public static void test_tree_set(){
        
        int n = 10;
        Random generator = new Random();
        Set<Integer> label_names_set = new TreeSet<>();
        for (int i=0; i<n; i++){
            
            int entry = generator.nextInt();
            label_names_set.add(entry);
            
        }
        
        int [] arr_label_names = new int [ n ];
        Arrays.fill(arr_label_names, INVALID_LABEL);
        int i = 0;
        for( Integer name : label_names_set ) {
          arr_label_names[i++] = name; //note the autounboxing here
        }
        
        int [] arr_label_names_sorted = new int [ n ];
        System.arraycopy(arr_label_names, 0, arr_label_names_sorted, 0, n);
        
        for(i=0; i<n; i++){
            System.out.println(arr_label_names_sorted[i] + " " + arr_label_names[i]);
            if(arr_label_names_sorted[i] != arr_label_names[i])
                System.err.println("Not sorted");
        }
        
    }
    
}
