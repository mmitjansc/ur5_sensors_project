
#include <stdio.h>
#include <math.h>
#include "portaudio.h"
#include "ros/ros.h"
#include <chrono>
#include <algorithm>    // std::max
#include <unistd.h>
#include <time.h>
#include <vector>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include "ur5_ros_arduino/pressures.h"
#include "std_msgs/Float32MultiArray.h"

#define NUM_SECONDS   (1.)
#define SAMPLE_RATE   (44100.)
#define FRAMES_PER_BUFFER  (44100)
#define TS (44100.);

#ifndef M_PI
#define M_PI  (3.14159265358979323)
#endif

#define TABLE_SIZE   (400) // For sine wave. Unrelated to 2nd order system
#define LENGTH (44100) // settling_time / SAMPLE_RATE


#define GAIN (1.)

struct syst {
	float a = 1.614157666028752e-06;
	float b = 1.614059996985478e-06;
	float c = -1.979555385907785;
	float d = 0.999818610557363;
} s;

struct timespec ts;

std::ofstream myfile;

namespace str {
	// Template to cast from anything to string
    template <typename T> std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
} 

class Impulse
{
public:
	
	PaStream *stream;
    float sine[TABLE_SIZE];
    int left_phase;
    int right_phase;
    char message[20];
	std::vector<float> outs = std::vector<float>(10);
	std::vector<int> ins = std::vector<int>(10);
	float impulse[LENGTH+1]; // impulse response
	int k;
	int j;
	int count;
	
	enum State {resting, grasping, holding};
	State state;

    Impulse() : stream(0), left_phase(0), right_phase(0), k(0), j(0), state(resting)
    {
        /* initialise sinusoidal wavetable */
        for( int i=0; i<TABLE_SIZE; i++ )
        {
            sine[i] = (float) 1.0*sin( ((double)i/(double)TABLE_SIZE) * M_PI * 2. );
        }

        sprintf( message, "No Message" );
        
        /* initialise 2nd order system wavetable */
        for( int i=0; i<LENGTH; i++ )
        {
            if (k == 0) {
				// dirac function at the start of the program
				k = 1;
				ins[0] = 1;
			}
			
			float output = (float) GAIN*(s.a*ins[1] + s.b*ins[2])*SAMPLE_RATE - s.c*outs[1] - s.d*outs[2];

			outs[0] = output;
			impulse[i] = output;
			
			std::rotate(ins.rbegin(), ins.rbegin() + 1, ins.rend());
			std::rotate(outs.rbegin(), outs.rbegin() + 1, outs.rend());
			
			ins[0] = 0; // To make sure the initial dirac value disappears when 'ins' is rotated
						
        }
        
        impulse[-1] = 0;

        sprintf( message, "No Message" );
    }

    bool open(PaDeviceIndex index)
    {
        PaStreamParameters outputParameters;

        outputParameters.device = index;
        if (outputParameters.device == paNoDevice) {
            return false;
        }

        const PaDeviceInfo* pInfo = Pa_GetDeviceInfo(index);
        if (pInfo != 0)
        {
            //printf("Output device name: '%s'\r", pInfo->name);
        }

        outputParameters.channelCount = 2;       /* stereo output */
        outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
        outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
        outputParameters.hostApiSpecificStreamInfo = NULL;

        PaError err = Pa_OpenStream(
            &stream,
            NULL, /* no input */
            &outputParameters,
            SAMPLE_RATE,
            FRAMES_PER_BUFFER,
            paClipOff,      /* we won't output out of range samples so don't bother clipping them */
            &Impulse::paCallback,
            this            /* Using 'this' for userData so we can cast to Impulse* in paCallback method */
            );

        if (err != paNoError)
        {
            /* Failed to open stream to device !!! */
            return false;
        }

        err = Pa_SetStreamFinishedCallback( stream, &Impulse::paStreamFinished );

        if (err != paNoError)
        {
            Pa_CloseStream( stream );
            stream = 0;

            return false;
        }

        return true;
    }

    bool close()
    {
        if (stream == 0)
            return false;

        PaError err = Pa_CloseStream( stream );
        stream = 0;

        return (err == paNoError);
    }


    bool start()
    {
        if (stream == 0)
            return false;

        PaError err = Pa_StartStream( stream );

        return (err == paNoError);
    }

    bool stop()
    {
        if (stream == 0)
            return false;

        PaError err = Pa_StopStream( stream );

        return (err == paNoError);
    }
    
    bool abort()
    {
        if (stream == 0)
            return false;

        PaError err = Pa_AbortStream( stream );

        return (err == paNoError);
    }
    
    void pressuresCallback(const ur5_ros_arduino::pressures::ConstPtr& msg) {
		
		std_msgs::Float32MultiArray dat = msg->vec;
		
		bool input = false;
		
		for (int i=0;i < dat.data.size(); i++) {
			
			if (dat.data[i] < 200) {
				input = true;
			}
		}
		
		//std::cout << "input: " << input << std::endl;
		
		int sleep_ms = 1000;
		
		switch (state) {
			
			case resting:
				//std::cout << "resting" << std::endl;
				if (input)  { state = grasping; }
				break;
				
			case grasping:
			
				//std::cout << "grasping" << std::endl;
				
				j = 0;
			
				//ROS_INFO("Resending signal");
				
				// Stop all current PortAudio callbacks and close previous stream:
				this->abort();
				this->close();
				
				
				if (this->open(Pa_GetDefaultOutputDevice()))
				{
					if (this->start())
					{
						printf("%d - Signal sent\n",count);
						count++;
						// Let's remove the sleep...
						//Pa_Sleep( sleep_ms );
						
						//this->stop();
					}

					// Remove close as well? Will this work?
					//this->close();
				}
			
				if (!input) { state = resting; } else {  state = holding; }
				
				break;
						
			case holding:
				//std::cout << "holding" << std::endl;
				if (!input) {state = resting; }
				
				break;
			
			default:
				//std::cout << "Something's wrong..." << std::endl;
				
				break;
				
			
		}
	}

private:
    /* The instance callback, where we have access to every method/variable in object of class Impulse */
    int paCallbackMethod(const void *inputBuffer, void *outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags)
    {
		
		float *out = (float*)outputBuffer;
        unsigned long i;

        (void) timeInfo; /* Prevent unused variable warnings. */
        (void) statusFlags;
        (void) inputBuffer;
        
        //std::cout << j << std::endl;

        for( i=0; i<framesPerBuffer; i++ )
        {
			
			//myfile << str::to_string(impulse[j]) << "\n";
			
			// LEFT:
            *out++ = sine[right_phase];  /* left */
            //*out++ = impulse[j]; // The computed output needs to be sent through audio jack 
            //*out++ = 0;
            
            // RIGHT:
            *out++ = sine[right_phase];  /* right */
            //*out++ = 0; // Output nothing
            //*out++ = impulse[j]; // The computed output needs to be sent through audio jack 
            
            
            int k = 8;
            
            left_phase += k;
            if( left_phase >= TABLE_SIZE ) left_phase -= TABLE_SIZE;
            right_phase += k; // higher pitch so we can distinguish left and right. 
            if( right_phase >= TABLE_SIZE ) right_phase -= TABLE_SIZE;
                    
            
            if (j < LENGTH) {
				j++;
			}
            
            /*
            auto t2 = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
            
            ts.tv_nsec = (long) std::max((150. - duration), 1.);
            int f = nanosleep(&ts,(struct timespec *)NULL); // Sleep for X nanoseconds
            
            auto t3 = std::chrono::high_resolution_clock::now();
			auto duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>( t3 - t1 ).count();
			if (f < 0.) {
				std::cout << "Loop duration: " << ts.tv_nsec << " ns\n";
			}*/
        }

        return paContinue;

    }

    /* This routine will be called by the PortAudio engine when audio is needed.
    ** It may called at interrupt level on some machines so don't do anything
    ** that could mess up the system like calling malloc() or free().
    */
    static int paCallback( const void *inputBuffer, void *outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void *userData )
    {
        /* Here we cast userData to Impulse* type so we can call the instance method paCallbackMethod, we can do that since 
           we called Pa_OpenStream with 'this' for userData */
        return ((Impulse*)userData)->paCallbackMethod(inputBuffer, outputBuffer,
            framesPerBuffer,
            timeInfo,
            statusFlags);
    }
    
    
    void paStreamFinishedMethod()
    {
        //printf( "Stream Completed: %s\n", message );
    }

    /*
     * This routine is called by portaudio when playback is done.
     */
    static void paStreamFinished(void* userData)
    {
        return ((Impulse*)userData)->paStreamFinishedMethod();
    }

    
};

class ScopedPaHandler
{
public:
    ScopedPaHandler()
        : _result(Pa_Initialize())
    {
    }
    ~ScopedPaHandler()
    {
        if (_result == paNoError)
        {
            Pa_Terminate();
        }
    }

    PaError result() const { return _result; }

private:
    PaError _result;
};

/*******************************************************************/
// Other initialization variables/functions:



/*******************************************************************/
int main(int argc, char **argv);
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "portaudio_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);
	
	Impulse imp;
	
	ros::Subscriber sub = nh.subscribe("/pressure", 1, &Impulse::pressuresCallback, &imp);		
	
	std::string s("ys.csv");
	myfile.open(s.c_str());
	
	ts.tv_sec = 0.;
    

    printf("PortAudio Test: output sine wave. SR = %f, BufSize = %d\n", SAMPLE_RATE, FRAMES_PER_BUFFER);
    
    ScopedPaHandler paInit;
    
    if( paInit.result() != paNoError ) goto error;
    
    //Initialize class counter variables
    imp.j = 0;
    imp.count = 1;	
	
	/*
    if (imp.open(Pa_GetDefaultOutputDevice()))
    {
        if (imp.start())
        {
            printf("Play for %f seconds.\n", NUM_SECONDS );
            Pa_Sleep( NUM_SECONDS * 1000 );

            imp.abort();
        }

        imp.close();
        myfile.close();    
		myfile.open(s.c_str());
    }
    printf("Test finished.\n");    
    */
    
    while (ros::ok()) {		
		
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	
	return paNoError;

error:
    fprintf( stderr, "An error occured while using the portaudio stream\n" );
    fprintf( stderr, "Error number: %d\n", paInit.result() );
    fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( paInit.result() ) );
    
    return 1;
}
