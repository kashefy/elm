/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import org.yaml.snakeyaml.Yaml;
import model.utils.files.FileIO;
import java.io.*;
import java.util.*;

public class FeatureMapParams extends AbstractParams{

    public static int ORIENT_INIT_MODE_ARBITRARY = -1;
    public static int ORIENT_INIT_MODE_RANGE     = -2;
    
    private int m_supportRadius;
    private int m_nofScales;
    private double m_scalingFactor;
    
    private int m_orientationInitMode;
    private double m_orientationResolution;
    private double [] m_orientationsInDeg;
    
    private double m_gaborFrequency;
    private double m_elongation;

    public double getElongation() {
        return m_elongation;
    }

    public void setElongation(double m_elongation) {
        this.m_elongation = m_elongation;
    }

    public double getGaborFrequency() {
        return m_gaborFrequency;
    }

    public void setGaborFrequency(double m_gaborFrequency) {
        this.m_gaborFrequency = m_gaborFrequency;
    }

    public int getNofScales() {
        return m_nofScales;
    }

    public void setNofScales(int m_nofScales) {
        this.m_nofScales = m_nofScales;
    }

    public int getOrientationInitMode() {
        return m_orientationInitMode;
    }

    public void setOrientationInitMode(int par_orientationInitMode) {
        this.m_orientationInitMode = par_orientationInitMode;
    }

    public double getOrientationResolution() {
        return m_orientationResolution;
    }

    public void setOrientationResolution(double m_orientationResolution) {
        this.m_orientationResolution = m_orientationResolution;
        m_orientationInitMode = ORIENT_INIT_MODE_RANGE;
    }

    public double[] getOrientationsInDeg() {
        return m_orientationsInDeg;
    }

    public void setOrientationsInDeg(double[] m_orientationsInDeg) {
        this.m_orientationsInDeg = m_orientationsInDeg;
        m_orientationInitMode = ORIENT_INIT_MODE_ARBITRARY;
    }
    
    public void setOrientation(double par_orientationResolution){
        
        setOrientationResolution(par_orientationResolution);
    }
    
    public void setOrientation(double[] par_orientationsInDeg){
        
        setOrientationsInDeg(par_orientationsInDeg);
    }

    public double getScalingFactor() {
        return m_scalingFactor;
    }

    public void setScalingFactor(double m_scalingFactor) {
        this.m_scalingFactor = m_scalingFactor;
    }

    public int getSupportRadius() {
        return m_supportRadius;
    }

    public void setSupportRadius(int m_supportRadius) {
        this.m_supportRadius = m_supportRadius;
    }
    
    private void load_yml(File par_file){
        
        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);
            m_supportRadius = (int) root.get("m_supportRadius"); 
            m_nofScales = (int) root.get("m_nofScales"); 
            m_scalingFactor = (double) root.get("m_scalingFactor"); 
            if( root.containsKey("m_orientationResolution") ){
                
                m_orientationResolution = (double) root.get("m_orientationResolution");  
                m_orientationInitMode = ORIENT_INIT_MODE_RANGE;
            }
            if( root.containsKey("m_orientationsInDeg") ){
                
                System.err.println("loading m_orientationsInDeg from file not supported yet.");
                m_orientationInitMode = ORIENT_INIT_MODE_ARBITRARY;
            }
            m_gaborFrequency = (double) root.get("m_gaborFrequency");  
            m_elongation = (double) root.get("m_elongation");
        }
        catch(Exception e){

            System.err.println("Error: " + e.getMessage());
        }
    }
    
    @Override
    public void save(String par_filepath){
        
        Yaml yaml = new Yaml();
        Map<String, Object> root = new HashMap<>();
        try(PrintWriter p = new PrintWriter(par_filepath)){

            root.put("m_supportRadius", m_supportRadius);
            root.put("m_nofScales", m_nofScales);
            root.put("m_scalingFactor", m_scalingFactor);
            root.put("m_orientationResolution", m_orientationResolution);
            root.put("m_gaborFrequency", m_gaborFrequency);
            root.put("m_elongation", m_elongation);
            
            p.println(yaml.dump(root));
            p.close();
        }
        catch(Exception e){
            System.err.println("Error: " + e.getMessage());
        }
    }
    
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
                if(strFieldIdentifier.equalsIgnoreCase("m_supportRadius")){
                    
                    m_supportRadius = Integer.parseInt(strFieldValue);
                }
                if(strFieldIdentifier.equalsIgnoreCase("m_nofScales")){
                    
                    m_nofScales = Integer.parseInt(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_scalingFactor")){
                    
                    m_scalingFactor = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_orientationResolution")){
                    
                    m_orientationResolution = Double.parseDouble(strFieldValue);
                    m_orientationInitMode = ORIENT_INIT_MODE_RANGE;
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_orientationsInDeg")){
                    
                    // still to implement
                    m_orientationInitMode = ORIENT_INIT_MODE_ARBITRARY;
                }   
                else if(strFieldIdentifier.equalsIgnoreCase("m_gaborFrequency")){
                    
                    m_gaborFrequency = Double.parseDouble(strFieldValue);
                }   
                else if(strFieldIdentifier.equalsIgnoreCase("m_elongation")){
                    
                    m_elongation = Double.parseDouble(strFieldValue);
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
    
}
