/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

/**
 *
 * @author woodstock
 */
import model.features.FeatureMapParams;
import model.utils.AbstractParams;
import model.utils.files.FileIO;
import org.yaml.snakeyaml.Yaml;
import java.io.*;
import java.util.*;

public class SaliencyParams extends AbstractParams{
    
    protected String m_strMainInputDir;
    
    protected int m_nofRows;
    protected int m_nofCols;
    protected double m_cutOffBelow_orientResponseVals;
    
    protected FeatureMapParams m_featMapParams;
    
    private void load_yml(File par_file){

        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);
                
            Map file_io = (Map) root.get("file I/O");
            m_strMainInputDir = (String) file_io.get("m_strMainInputDir");

            Map data = (Map) root.get("data");
            m_nofRows = (int) data.get("m_nofRows");
            m_nofCols = (int) data.get("m_nofRows");
            
            m_cutOffBelow_orientResponseVals = (double) root.get("m_cutOffBelow_orientResponseVals");
            
            Map parameter_objects = (Map) root.get("parameter objects");
            Set<String> keys = parameter_objects.keySet();
            for(String key : keys){

                String str_keyValue = (String) parameter_objects.get(key);
                int index = str_keyValue.lastIndexOf('.'); // determine if key value is a file name
                if( index > 0 ){

                    File parameter_file = new File( str_keyValue );
                    String parameter_file_path;
                    if(! parameter_file.isAbsolute() ){

                        parameter_file_path = new File( m_strMainInputDir, str_keyValue ).getPath();
                    }
                    else{
                        parameter_file_path = str_keyValue;
                    }
                    AbstractParams param_obj = null;
                    switch(key){

                        case "m_featParams":
                            param_obj = m_featMapParams = new FeatureMapParams();
                            break;
                        default:
                            System.err.println("Unknown parameter object type: "+key);
                    }
                    if( param_obj != null ){

                        param_obj.load( parameter_file_path );
                    }
                }
                else{
                    System.err.println("Unknown vlaue type for parameter objects::"+key);
                }
            }
        }
        catch(ClassCastException | NullPointerException e){

            System.err.println("Error while parsing: " + e.getMessage());
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
                if (!strLine.startsWith("//")){
                    
                    arrLineElements = strLine.split(strFieldDelimiter);

                    strFieldIdentifier = arrLineElements[0];
                    strFieldValue = arrLineElements[1];

                    if(strFieldIdentifier.equalsIgnoreCase("m_strMainInputDir")){

                            m_strMainInputDir = strFieldValue;
                            if (!m_strMainInputDir.endsWith(File.separator))
                                m_strMainInputDir += File.separator;
                        }
                    if(strFieldIdentifier.equalsIgnoreCase("m_nofRows")){

                        m_nofRows = Integer.parseInt(strFieldValue);
                    }
                    if(strFieldIdentifier.equalsIgnoreCase("m_nofCols")){

                        m_nofCols = Integer.parseInt(strFieldValue);
                    }
                    if(strFieldIdentifier.equalsIgnoreCase("m_cutOffBelow_orientResponseVals")){

                        m_cutOffBelow_orientResponseVals = Double.parseDouble(strFieldValue);
                        if(m_cutOffBelow_orientResponseVals < 0)
                            System.err.println("Encountered negative value for SaliencyParams::m_cutOffBelow_orientResponseVals. Please revise.");
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_featParams")){

                            m_featMapParams = new FeatureMapParams();
                            if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                                m_featMapParams.load(m_strMainInputDir + strFieldValue);
                            }
                            else
                            m_featMapParams.load(strFieldValue);
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
    
    public SaliencyParams(){
        
    }
}
