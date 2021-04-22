Hi guys! 

I've included all the code that does the frequency measurement. Please read for more information, as certain things are crucial for the implementation to work. Each section starts with the description and is followed by the code. 
	1.Creating a test signal
	2.Initializing requirements in main BEFORE while loop
	3.While loop body
	4.Helper function for obtaining frequency

1.Creating a test signal:

The below is code for creating a test signal. The array size is defined by 48000/1500. 480000 is the sample rate. 
Most audio is sampled at 44.1kHz or 48kHz. I chose the latter. Moreoever, 1500 is the desired frequency. This is
VERY IMPORTANT: the array size must be a power of 2 (i.e. 32,64,256,512,1024,...,8192) The audio signal from the mic
should be of power 2. For testing, ensure that you use a frequency that properly divides 48000 into a power of 2. 
##################################################################################################################
		int test_frequency = 750;
		
		int arraySize = 48000/test_frequency;
		int test_size = arraySize*sizeof(int);
		int* test = malloc(arraySize*sizeof(int));

		for (int i = 0; i < arraySize; i++) {
			test[i] = (int)((arm_sin_f32(i*2*pi/arraySize) + 1)*(4095/2));
		}
##################################################################################################################

2.Initializing requirements in main BEFORE while loop:

FFT can only accomodate for powers of 2. Make sure that you define the size of the array to reflect that. In my case,
I have chosen to go with 2048. But, again, 512, 1024, 2046, etc. are entirely valid. The fft input and output buffers 
are what we will use for the actual FFT computation. Make sure to declare a handler: think of it as the object that does
the computation. 
##################################################################################################################
		float fft_in_buf[2048];
		float fft_out_buf[2048];

		arm_rfft_fast_instance_f32 fft_handler;
		arm_rfft_fast_init_f32(&fft_handler, 2048);
##################################################################################################################

3.While loop body:

This block is fairly simple, all we're doing is assigning the input signal values to the FFT input buffer. Make sure to 
use a modulus operator to wrap back around to the begin of the test array so as to avoid HARD FAULTS. Once the input buffer
is full, we can get frequencies.
##################################################################################################################
		int fft_in_ptr = 0;

			  for(int point = 0; point < 2048; point++){
				  int max_index = test_size/sizeof(test[0]);
				  fft_in_buf[fft_in_ptr] = test[point%max_index];
				  fft_in_ptr++;
			  }
			   getFrequencies(&fft_handler, fft_in_buf, fft_out_buf);			
##################################################################################################################

4.Helper function for obtaining frequency: 

This is our bread and butter. Use this to calculate the frequencies AND return the highest frequency value. 
Frequencies will always be HALF the size of the the original FFT input anf FFT output buffers. 
This is because the arm_rfft_fast_f32() method stores 2 coefficients into the 
output buffer, namely the real and imaginary components of the result. These 2 values are ADJACENT to each other. 
Output[i] would correspond to the real value and Output[i+1] would correspond to the imaginary. From there, we simply
find the magnitude. Notice that the for loop increments by 2 to accomodate do both components. Use noise to normalize 
frequency values. 

OPTIONAL: We use the DB scale to measure the loudness of a tone that is more readible, but that is not necessary. Any thresholding
method will do. 

VERY IMPORTANT: to actually view the presence of all frequency, we must perform the following operation to find the index to 
look at within the frequencies array. 

	Frequency_index = desired_frequency x 2 x 1024/sample_rate
	
Remember that we have chosen 48000 as our sampling rate. If our desired frequency is 1500 then: 
	
	Frequency_index = 1500 x 2 x 1025/ 48000 = 64.0625
	
So if we want to check for 1500 Hz, check frequencies array at frequencies[64]. All other values will be zero (For ideal sine) 
	
##################################################################################################################
		int getFrequencies(arm_rfft_fast_instance_f32* handler, int sampling_rate ,float* buffer_in, float* buffer_out) {

			arm_rfft_fast_f32(handler, buffer_in, buffer_out,0);

			int frequencies[1024];
			int frequency_index = 0;
			int noise = 150; //variable noise offset
			int max = 0;
			int temp_max = 0;
			int frequency = 0;
			//calculate abs values and linear-to-dB
			for (int i=0; i<2048; i=i+2) {

				int real = buffer_out[i];
				int complex = buffer_out[i+1];
				int result = sqrt(real*real+complex*complex);

				frequencies[frequency_index] = (int)(20*log10f(result));
				if (frequencies[frequency_index]<0) frequencies[frequency_index]=0;

				temp_max = frequencies[frequency_index];

				if(frequency_index > 0 && temp_max > max){
					max = temp_max;
					frequency = (int) frequency_index*sampling_rate/2048;
				}
				frequency_index++;
			}
			return frequency;
		}
##################################################################################################################
