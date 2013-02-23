/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.simulation;

/**
 *
 * @author woodstock
 */
import model.utils.Stopwatch;
import model.utils.files.FileIO;
import java.io.*;

public class SimulationTune {
    
    public static void tune_MNIST_layerF_onOff_learningRateZ(){
        
        String tune_suffix = "learningRateZ";
        String output_dir_name = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File tune_log_ile = new File(output_dir_name, "tune_log_"+tune_suffix+".txt"); 
        try(PrintWriter pw = new PrintWriter(new FileWriter(tune_log_ile))){

            double start_tune_value = 0.01;
            double end_tune_value = 1.0;
            double tune_increment = 0.10;
            int tune_iteration = 0;
            int nof_rep = 5;
            for(double tune_value = start_tune_value; tune_value < end_tune_value; tune_value += tune_increment, tune_iteration++){

                for(int rep = 0; rep<nof_rep; rep++){
                    
                    pw.print(tune_iteration+","+tune_value);
                    System.out.println("Tuning at "+tune_value);
                    Stopwatch timer = new Stopwatch();
                    timer.start();

                    String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
                    File input_dir = new File(str_main_input_dir);
                    AbstractSimulation simulation;

                    simulation = new SimulationMNIST_layerF_onOff();

                    SimulationParams sim_params = new SimulationParams();
                    sim_params.load(new File(input_dir, "simParamFile_layerF.yml").getPath());

                    File main_output_dir = new File(sim_params.getMainOutputDir());
                    File tune_dir = new File(main_output_dir, FileIO.DIR_NAME_TUNE+"_"+tune_suffix+"_"+tune_iteration);
                    boolean dir_exists;
                    if(!tune_dir.exists()){
                        dir_exists = tune_dir.mkdir();
                        if(!dir_exists){
                            System.err.println("Failed to create directory " + tune_dir.getPath());
                        }
                    }
                    String[] arr_sub_dir_names = new String[]{FileIO.DIR_NAME_ACTIVITY_LAYER_F, 
                        FileIO.DIR_NAME_ACTIVITY_LAYER_Z, 
                        FileIO.DIR_NAME_ATTENTION, 
                        FileIO.DIR_NAME_WATCH, 
                        FileIO.FILTER_RESPONSE};
                    for ( String sub_dir_name : arr_sub_dir_names ){

                        File sub_dir = new File(tune_dir, sub_dir_name);
                        if(!sub_dir.exists()){
                            dir_exists = sub_dir.mkdir();
                            if(!dir_exists){
                                System.err.println("Failed to create sub-directory " + sub_dir.getPath());
                            }
                        }
                    }

                    sim_params.setMainOutputDir(tune_dir.getPath());

                    sim_params.set_log_append( rep>0 );
                    sim_params.getLearnerParamsRef().setLearningRateEtaInitVal(tune_value);

                    simulation.setParams(sim_params);


                    System.out.print("init()...");
                    simulation.init();
                    System.out.print("done.\n");

                    simulation.run();

                    timer.stop();
                    long elapsed_time_seconds = timer.getElapsedTime()/1000;
                    int elapsed_time_minutes = (int)elapsed_time_seconds/60;
                    elapsed_time_seconds = elapsed_time_seconds % 60;
                    System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
                    pw.println(); 

                }
            }
            pw.close();
         }
        catch (Exception e){

             System.err.println("Error: " + e.getMessage());
        }
    }
    
    public static void tune_MNIST_layerF_onOff_learningRateF(){
        
        String tune_suffix = "learningRateF";
        String output_dir_name = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File tune_log_ile = new File(output_dir_name, "tune_log_"+tune_suffix+".txt"); 
        try(PrintWriter pw = new PrintWriter(new FileWriter(tune_log_ile))){

            double start_tune_value = 0.01;
            double end_tune_value = 1.0;
            double tune_increment = 0.10;
            int tune_iteration = 0;
            int nof_rep = 5;
            for(double tune_value = start_tune_value; tune_value < end_tune_value; tune_value += tune_increment, tune_iteration++){

                for(int rep = 0; rep<nof_rep; rep++){   
                    pw.print(tune_iteration+","+tune_value);
                    System.out.println("Tuning at "+tune_value);
                    Stopwatch timer = new Stopwatch();
                    timer.start();

                    String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
                    File input_dir = new File(str_main_input_dir);
                    AbstractSimulation simulation;

                    simulation = new SimulationMNIST_layerF_onOff();

                    SimulationParams sim_params = new SimulationParams();
                    sim_params.load(new File(input_dir, "simParamFile_layerF.yml").getPath());

                    File main_output_dir = new File(sim_params.getMainOutputDir());
                    File tune_dir = new File(main_output_dir, FileIO.DIR_NAME_TUNE+"_"+tune_suffix+"_"+tune_iteration);
                    boolean dir_exists;
                    if(!tune_dir.exists()){
                        dir_exists = tune_dir.mkdir();
                        if(!dir_exists){
                            System.err.println("Failed to create directory " + tune_dir.getPath());
                        }
                    }
                    String[] arr_sub_dir_names = new String[]{FileIO.DIR_NAME_ACTIVITY_LAYER_F, 
                        FileIO.DIR_NAME_ACTIVITY_LAYER_Z, 
                        FileIO.DIR_NAME_ATTENTION, 
                        FileIO.DIR_NAME_WATCH, 
                        FileIO.FILTER_RESPONSE};
                    for ( String sub_dir_name : arr_sub_dir_names ){

                        File sub_dir = new File(tune_dir, sub_dir_name);
                        if(!sub_dir.exists()){
                            dir_exists = sub_dir.mkdir();
                            if(!dir_exists){
                                System.err.println("Failed to create sub-directory " + sub_dir.getPath());
                            }
                        }
                    }

                    sim_params.setMainOutputDir(tune_dir.getPath());

                    sim_params.set_log_append( rep>0 );
                    sim_params.getLearnerParams_layerF_Ref().setLearningRateEtaInitVal(tune_value);

                    simulation.setParams(sim_params);

                    System.out.print("init()...");
                    simulation.init();
                    System.out.print("done.\n");

                    simulation.run();

                    timer.stop();
                    long elapsed_time_seconds = timer.getElapsedTime()/1000;
                    int elapsed_time_minutes = (int)elapsed_time_seconds/60;
                    elapsed_time_seconds = elapsed_time_seconds % 60;
                    System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
                    pw.println(); 
                }     
            }
            pw.close();
        }
        catch (Exception e){

             System.err.println("Error: " + e.getMessage());
        }
    }
    
    public static void tune_MNIST_layerF_onOff_wta_max_rate_factor_layerF(){
        
        String tune_suffix = "wta_max_rate_factor_layerF";
        String output_dir_name = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File tune_log_ile = new File(output_dir_name, "tune_log_"+tune_suffix+".txt"); 
        try(PrintWriter pw = new PrintWriter(new FileWriter(tune_log_ile))){

            double start_tune_value = 7.5;
            double end_tune_value = 0.01;
            double tune_increment = -1.00;
            int tune_iteration = 0;
            int nof_rep = 5;
            for(double tune_value = start_tune_value; tune_value > end_tune_value; tune_value += tune_increment, tune_iteration++){

                for(int rep = 0; rep<nof_rep; rep++){
                    
                    pw.print(tune_iteration+","+tune_value);
                    System.out.println("Tuning at "+tune_value);
                    Stopwatch timer = new Stopwatch();
                    timer.start();

                    String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
                    File input_dir = new File(str_main_input_dir);
                    AbstractSimulation simulation;

                    simulation = new SimulationMNIST_layerF_onOff();

                    SimulationParams sim_params = new SimulationParams();
                    sim_params.load(new File(input_dir, "simParamFile_layerF.yml").getPath());

                    File main_output_dir = new File(sim_params.getMainOutputDir());
                    File tune_dir = new File(main_output_dir, FileIO.DIR_NAME_TUNE+"_"+tune_suffix+"_"+tune_iteration);
                    boolean dir_exists;
                    if(!tune_dir.exists()){
                        dir_exists = tune_dir.mkdir();
                        if(!dir_exists){
                            System.err.println("Failed to create directory " + tune_dir.getPath());
                        }
                    }
                    String[] arr_sub_dir_names = new String[]{FileIO.DIR_NAME_ACTIVITY_LAYER_F, 
                        FileIO.DIR_NAME_ACTIVITY_LAYER_Z, 
                        FileIO.DIR_NAME_ATTENTION, 
                        FileIO.DIR_NAME_WATCH, 
                        FileIO.FILTER_RESPONSE};
                    for ( String sub_dir_name : arr_sub_dir_names ){

                        File sub_dir = new File(tune_dir, sub_dir_name);
                        if(!sub_dir.exists()){
                            dir_exists = sub_dir.mkdir();
                            if(!dir_exists){
                                System.err.println("Failed to create sub-directory " + sub_dir.getPath());
                            }
                        }
                    }

                    sim_params.setMainOutputDir(tune_dir.getPath());

                    sim_params.set_log_append( rep>0 );
                    sim_params.getCompetitionParams_layerF_ref().setMaxRateFactor(tune_value);

                    simulation.setParams(sim_params);



                    System.out.print("init()...");
                    simulation.init();
                    System.out.print("done.\n");

                    simulation.run();

                    timer.stop();
                    long elapsed_time_seconds = timer.getElapsedTime()/1000;
                    int elapsed_time_minutes = (int)elapsed_time_seconds/60;
                    elapsed_time_seconds = elapsed_time_seconds % 60;
                    System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
                    pw.println(); 
                }       
            }
            pw.close();
         }
        catch (Exception e){

             System.err.println("Error: " + e.getMessage());
        }
    }
    
    public static void tune_MNIST_layerF_onOff_wta_max_rate_factor_layerZ(){
        
        String tune_suffix = "wta_max_rate_factor_layerZ";
        String output_dir_name = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File tune_log_ile = new File(output_dir_name, "tune_log_"+tune_suffix+".txt"); 
        try(PrintWriter pw = new PrintWriter(new FileWriter(tune_log_ile))){

            double start_tune_value = 7.5;
            double end_tune_value = 0.01;
            double tune_increment = -1.00;
            int tune_iteration = 0;
            int nof_rep = 5;
            for(double tune_value = start_tune_value; tune_value > end_tune_value; tune_value += tune_increment, tune_iteration++){

                for(int rep = 0; rep<nof_rep; rep++){
                    
                    pw.print(tune_iteration+","+tune_value);
                    System.out.println("Tuning at "+tune_value);
                    Stopwatch timer = new Stopwatch();
                    timer.start();

                    String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
                    File input_dir = new File(str_main_input_dir);
                    AbstractSimulation simulation;

                    simulation = new SimulationMNIST_layerF_onOff();

                    SimulationParams sim_params = new SimulationParams();
                    sim_params.load(new File(input_dir, "simParamFile_layerF.yml").getPath());

                    File main_output_dir = new File(sim_params.getMainOutputDir());
                    File tune_dir = new File(main_output_dir, FileIO.DIR_NAME_TUNE+"_"+tune_suffix+"_"+tune_iteration);
                    boolean dir_exists;
                    if(!tune_dir.exists()){
                        dir_exists = tune_dir.mkdir();
                        if(!dir_exists){
                            System.err.println("Failed to create directory " + tune_dir.getPath());
                        }
                    }
                    String[] arr_sub_dir_names = new String[]{FileIO.DIR_NAME_ACTIVITY_LAYER_F, 
                        FileIO.DIR_NAME_ACTIVITY_LAYER_Z, 
                        FileIO.DIR_NAME_ATTENTION, 
                        FileIO.DIR_NAME_WATCH, 
                        FileIO.FILTER_RESPONSE};
                    for ( String sub_dir_name : arr_sub_dir_names ){

                        File sub_dir = new File(tune_dir, sub_dir_name);
                        if(!sub_dir.exists()){
                            dir_exists = sub_dir.mkdir();
                            if(!dir_exists){
                                System.err.println("Failed to create sub-directory " + sub_dir.getPath());
                            }
                        }
                    }

                    sim_params.setMainOutputDir(tune_dir.getPath());

                    sim_params.set_log_append( rep>0 );
                    sim_params.getCompetitionParamsRef().setMaxRateFactor(tune_value);

                    simulation.setParams(sim_params);

                    System.out.print("init()...");
                    simulation.init();
                    System.out.print("done.\n");

                    simulation.run();

                    timer.stop();
                    long elapsed_time_seconds = timer.getElapsedTime()/1000;
                    int elapsed_time_minutes = (int)elapsed_time_seconds/60;
                    elapsed_time_seconds = elapsed_time_seconds % 60;
                    System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
                    pw.println(); 
                }
            }         
            pw.close();
        }
        catch (Exception e){

             System.err.println("Error: " + e.getMessage());
        }
    }
    
    public static void tune_MNIST_layerF_onOff(){
        
        String output_dir_name = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File tune_log_ile = new File(output_dir_name, "tune_log"+".txt"); 
        try(PrintWriter pw = new PrintWriter(new FileWriter(tune_log_ile))){

            double start_tune_value = 1;
            double end_tune_value = 0.01;
            double tune_increment = -1;
            int nof_rep = 1;
            int tune_iteration = 0;
            for(double tune_value = start_tune_value; tune_value > end_tune_value; tune_value += tune_increment, tune_iteration++){

                for(int rep = 0; rep<nof_rep; rep++){
                    
                    pw.print(rep+",");

                    pw.print(tune_iteration+","+tune_value);
                    System.out.println("Tuning at "+tune_value);
                    Stopwatch timer = new Stopwatch();
                    timer.start();

                    String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
                    File input_dir = new File(str_main_input_dir);
                    AbstractSimulation simulation;

                    simulation = new SimulationMNIST_layerF_onOff_loadF_indepLayer();

                    SimulationParams sim_params = new SimulationParams();
                    sim_params.load(new File(input_dir, "simParamFile_layerF.yml").getPath());

                    File main_output_dir = new File(sim_params.getMainOutputDir());
                    File tune_dir = new File(main_output_dir, FileIO.DIR_NAME_TUNE+"_"+tune_iteration);
                    boolean dir_exists;
                    if(!tune_dir.exists()){
                        dir_exists = tune_dir.mkdir();
                        if(!dir_exists){
                            System.err.println("Failed to create directory " + tune_dir.getPath());
                        }
                    }
                    String[] arr_sub_dir_names = new String[]{FileIO.DIR_NAME_ACTIVITY_LAYER_F, 
                        FileIO.DIR_NAME_ACTIVITY_LAYER_Z, 
                        FileIO.DIR_NAME_ATTENTION, 
                        FileIO.DIR_NAME_WATCH, 
                        FileIO.FILTER_RESPONSE};
                    for ( String sub_dir_name : arr_sub_dir_names ){

                        File sub_dir = new File(tune_dir, sub_dir_name);
                        if(!sub_dir.exists()){
                            dir_exists = sub_dir.mkdir();
                            if(!dir_exists){
                                System.err.println("Failed to create sub-directory " + sub_dir.getPath());
                            }
                        }
                    }

                    sim_params.setMainOutputDir(tune_dir.getPath());

                    sim_params.set_log_append( rep>0 );

                    // set tuning value here
                    simulation.setParams(sim_params);

                    System.out.print("init()...");
                    simulation.init();
                    System.out.print("done.\n");

                    simulation.run();

                    timer.stop();
                    long elapsed_time_seconds = timer.getElapsedTime()/1000;
                    int elapsed_time_minutes = (int)elapsed_time_seconds/60;
                    elapsed_time_seconds = elapsed_time_seconds % 60;
                    System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
                    pw.println(); 
                }
            }
            pw.close();
        }
        catch (Exception e){

             System.err.println("Error: " + e.getMessage());
        }
    }
}
