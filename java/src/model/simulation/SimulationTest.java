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
import java.io.*;

public class SimulationTest {
    
    public static void testAttSeq(){
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        AbstractSimulation simulation;
        
        simulation = new SimulationSceneSampler();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(strMainInputDir + "simParamFile.txt");
        
        simulation.setParams(simParams);
        
        simulation.init();
        System.out.println("init()...done");
        
        simulation.run();
    }
    
    public static void testSimplePatterns(){
        
        String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\barSet\\";
        File main_input_dir = new File(str_main_input_dir);
        AbstractSimulation simulation;
        
        //simulation = new SimulationSimplePatterns();
        simulation = new SimulationBarSet();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(new File(main_input_dir, "simParamFile.txt").getPath());
        
        simulation.setParams(simParams);
        
        simulation.init();
        System.out.println("init()...done");
        
        simulation.run();
    }
    
    public static void testMNIST(){
        
        Stopwatch timer = new Stopwatch();
        timer.start();
        
        String str_main_input_dir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\txt";
        File main_input_dir = new File(str_main_input_dir);
        AbstractSimulation simulation;
        
        simulation = new SimulationMNIST();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(new File(main_input_dir, "simParamFile.txt").getPath());
        
        simulation.setParams(simParams);
        
        System.out.print("init()...");
        simulation.init();
        System.out.print("done.\n");
        
        simulation.run();
        
        timer.stop();
        long elapsed_time_seconds = timer.getElapsedTime()/1000;
        int elapsed_time_minutes = (int)elapsed_time_seconds/60;
        elapsed_time_seconds = elapsed_time_seconds % 60;
        System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
    }
    
    public static void testMNISTInterm(){
        
        Stopwatch timer = new Stopwatch();
        timer.start();
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        AbstractSimulation simulation;
        
        simulation = new SimulationMNISTInterm();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(strMainInputDir + "simParamFile.txt");
        
        simulation.setParams(simParams);
        
        simulation.init();
        System.out.println("init()...done");
        
        simulation.run();
        
        timer.stop();
        System.out.println(timer.getElapsedTime());   
    }
    
    public static void testMNIST_layerF(){
        
        Stopwatch timer = new Stopwatch();
        timer.start();
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        File input_dir = new File(strMainInputDir);
        AbstractSimulation simulation;
        
        simulation = new SimulationMNIST_layerF();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(new File(input_dir, "simParamFile_layerF.yml").getPath());
        
        simulation.setParams(simParams);
        
        System.out.print("init()...");
        simulation.init();
        System.out.print("done.\n");
        
        simulation.run();
        
        timer.stop();
        long elapsed_time_seconds = timer.getElapsedTime()/1000;
        int elapsed_time_minutes = (int)elapsed_time_seconds/60;
        elapsed_time_seconds = elapsed_time_seconds % 60;
        System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
   
    }
    
    public static void test_MNIST_layerIndep(){
        
        Stopwatch timer = new Stopwatch();
        timer.start();
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        File input_dir = new File(strMainInputDir);
        AbstractSimulation simulation;
        
        simulation = new Simulation_MNIST_layerIndep();
        
        SimulationParams simParams = new SimulationParams();
        simParams.load(new File(input_dir, "simParamFile_layerF.yml").getPath());
        
        simulation.setParams(simParams);
        
        System.out.print("init()...");
        simulation.init();
        System.out.print("done\n");
        
        simulation.run();
        
        timer.stop();
        long elapsed_time_seconds = timer.getElapsedTime()/1000;
        int elapsed_time_minutes = (int)elapsed_time_seconds/60;
        elapsed_time_seconds = elapsed_time_seconds % 60;
        System.out.println(elapsed_time_minutes + " min " + elapsed_time_seconds + " sec");
   
    }
}
