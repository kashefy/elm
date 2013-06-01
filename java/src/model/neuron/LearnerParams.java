/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.neuron;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import model.utils.files.FileIO;
import org.yaml.snakeyaml.Yaml;
import java.io.*;
import java.util.*;

public class LearnerParams extends AbstractParams{
    
    // architecture/topology parameters
    private int m_nofInputNodes;
    
    // learning parameters
    private double m_learningRateEtaInitVal;
   
    private int m_historyLength;
    
    // weight parameters
    private double[] m_initialWeights;
    private boolean m_bInitRandWeights;
    private double m_scaleRandWeights;
    private double m_nBias;
    private boolean m_bBiasSet;
    private double m_nWeightLimit;
    
    public void setWeightParams(boolean par_bInitRandomWeights, double par_scale, 
            double[] par_initialWeights){
        
        m_bInitRandWeights      = par_bInitRandomWeights;
        m_scaleRandWeights      = par_scale;
        m_initialWeights        = par_initialWeights;
    }
    
    public void setWeightParams(boolean par_bInitRandomWeights, double par_scale, 
            double[] par_initialWeights, double par_nBias){
        
        m_bInitRandWeights      = par_bInitRandomWeights;
        m_scaleRandWeights      = par_scale;
        m_initialWeights        = par_initialWeights;
        m_nBias                 = par_nBias;
        m_bBiasSet              = true;
    }
    
    public boolean initWithRandomWeights(){
        
        return m_bInitRandWeights;
    }
    
    public double getScaleOfRandWeights(){
        
        return m_scaleRandWeights;
    }
    
    public int getHistoryLength() {
        
        return m_historyLength;
    }

    public void setHistoryLength(int par_historyLength) {
        
        this.m_historyLength = par_historyLength;
    }

    public double getLearningRateEtaInitVal() {
        
        return m_learningRateEtaInitVal;
    }

    public void setLearningRateEtaInitVal(double par_learningRateEtaInitVal) {
        
        this.m_learningRateEtaInitVal = par_learningRateEtaInitVal;
    }

    public int getNofInputNodes() {
        
        return m_nofInputNodes;
    }

    public void setNofInputNodes(int par_nofInputNodes) {
        
        this.m_nofInputNodes = par_nofInputNodes;
    }

    public double[] getWeights() {
        return m_initialWeights;
    }
    
    public double getBias(){
        
        return m_nBias;
    }
    
    public boolean wasBiasSet(){
        
        return m_bBiasSet;
    }

    public double getWeightLimit() {
        return m_nWeightLimit;
    }

    public void setWeightLimit(double par_nWeightLimit) {
        this.m_nWeightLimit = par_nWeightLimit;
    }
    
    /**
     * @brief load parameters from yaml file
     * @param file path 
     */
    private void load_yml(File par_file){

        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);
            
            if( root.containsKey("m_nofInputNodes") ){

                System.err.println("m_nofInputNodes inititalized at runtime. Skip.");
            }
            m_learningRateEtaInitVal = (double) root.get("m_learningRateEtaInitVal");
            m_historyLength = (int) root.get("m_historyLength");
            
            Map weight_parameters = (Map) root.get("weights");
            
            m_bBiasSet = weight_parameters.containsKey("m_nBias");
            if( m_bBiasSet ){
                
                m_nBias = (double) weight_parameters.get("m_nBias");
            }
            m_bInitRandWeights = (boolean) weight_parameters.get("m_bInitRandWeights");
            m_scaleRandWeights = (double) weight_parameters.get("m_scaleRandWeights");
            m_nWeightLimit = (double) weight_parameters.get("m_nWeightLimit");
                
            if(weight_parameters.containsKey("m_initialWeights")){
                
                Exception e = new UnsupportedOperationException("loading m_initialWeights from file not supported yet.");
                throw e;
            }
        }
        catch(Exception e){

            System.err.println("Error: " + e.getMessage());
        }
    }
    
    /**
     * @brief load from old text file
     * @param file path
     */
    private void load_txt(File par_file){
        
        try(Scanner scanner = new Scanner(par_file)){
        
            String strEntryDelimiter = "\n";
            String strFieldDelimiter = "=";
            scanner.useDelimiter(strEntryDelimiter);
            String strLine;
            String [] arrLineElements;
            String strFieldIdentifier;
            String strFieldValue;
            
            while(scanner.hasNext()){
                
                strLine = scanner.next();
                strLine = strLine.replaceAll("\r", "");
                arrLineElements = strLine.split(strFieldDelimiter);
                
                strFieldIdentifier = arrLineElements[0];
                strFieldValue = arrLineElements[1];
                if(strFieldIdentifier.equalsIgnoreCase("m_nofInputNodes")){
                    
                    //m_nofInputNodes = Integer.parseInt(strFieldValue);
                    // set at runtime
                }
                if(strFieldIdentifier.equalsIgnoreCase("m_learningRateEtaInitVal")){
                    
                    m_learningRateEtaInitVal = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_historyLength")){
                    
                    m_historyLength = Integer.parseInt(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_nBias")){
                    
                    m_nBias = Double.parseDouble(strFieldValue);
                    m_bBiasSet = true;
                }                
                else if(strFieldIdentifier.equalsIgnoreCase("m_bInitRandWeights")){
                    
                    m_bInitRandWeights = Boolean.parseBoolean(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_scaleRandWeights")){
                    
                    m_scaleRandWeights = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_nWeightLimit")){
                    
                    m_nWeightLimit = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_initialWeights")){
                    
                    if(!strFieldValue.isEmpty()){
                        
                        Exception e = new UnsupportedOperationException("loading m_initialWeights from file not supported yet.");
                        throw e;
                    }
                }           
            }
            scanner.close();
            
        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
        }
    }

    @Override
    public void load(String par_strFilename){
        
        File file = new File(par_strFilename);
        if(FileIO.extension( file.getPath()).equalsIgnoreCase("yml") ){
            
            load_yml(file);
        }
        else{
            load_txt(file);
        }
    }
    
    @Override
    public void save(String par_str_filepath){
        
        throw new UnsupportedOperationException("Saving to file not supported.");
    }

    public LearnerParams() {
        
        m_bInitRandWeights = true;
        m_scaleRandWeights = 0.01;
        m_bBiasSet = false;
        m_nWeightLimit = Double.MAX_VALUE;
    }

}
