/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils;

    
/*      
 * A class to help benchmark code
 * It simulates a real stop watch 
 * http://java.sun.com/docs/books/performance/1st_edition/html/JPMeasurement.fm.html
 *    
 */
public class Stopwatch {

    private long startTime  = -1;
    private long stopTime   = -1;
    private boolean running = false;

    public Stopwatch start() {
        
       startTime = System.currentTimeMillis();
       running = true;
       return this;
    }
    
    public Stopwatch stop() {
        
       stopTime = System.currentTimeMillis();
       running = false;
       return this;
    }
    
    /** returns elapsed time in milliseconds
      * if the watch has never been started then
      * return zero
      */
    public long getElapsedTime() {
        
       if (startTime == -1) {
          return 0;
       }
       if (running){
        
           return System.currentTimeMillis() - startTime;
           
       } else {
       
           return stopTime-startTime;
       } 
       
    }

    public Stopwatch reset() {
        
       startTime = -1;
       stopTime = -1;
       running = false;
       return this;
    }
    
    /*
     * Override to append elapsed time
     */
    @Override
    public String toString(){
        
        return super.toString() + "," + Long.toString(getElapsedTime());
    }
}