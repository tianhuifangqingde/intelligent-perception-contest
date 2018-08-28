# intelligent perception contest
intelligent这个文件夹的程序是放在ODROID里的，用来进行图像处理的。
仿真初赛包含图像处理和飞行控制。飞行控制为主线程，图像处理用新的线程处理，这样保证了无人机的控制频率，如果都在一个线程处理，控制频率达不到25Hz，效果会很差。
