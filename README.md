# Heat_Control_Project
This's a project based on arm-stm32f103.  
The main functions in this project are:  
  
    step_motor_control: controlling the step motor's direction, micro step and the count of steps  
                     and I control it by a task which is motorTask.  
    ADC: convert the analog, the voltage, to digital, the value of temp.  
                     and I implement it by a task which is adcTask.  
    print temp by serial helper: by printfTask.  
    
now it's still need add extra functions, the most important task is to implement the auto PID control,  
by now, I have achieved the common PID control, I want to improve it by Fuzzy to achieve the auto PID  
control, although I still don't make the Fuzzy rules clearly, but I will learn it last, I hope which   
can be done as soon as possible! If someone has such experiences, please give me some advice, thanks!
