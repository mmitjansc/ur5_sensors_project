/** @file paex_sine.c
	@ingroup examples_src
	@brief Play a sine wave for several seconds.
	@author Ross Bencina <rossb@audiomulch.com>
    @author Phil Burk <philburk@softsynth.com>
*/
/*
 * $Id: paex_sine.c 1752 2011-09-08 03:21:55Z philburk $
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com/
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The text above constitutes the entire PortAudio license; however, 
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also 
 * requested that these non-binding requests be included along with the 
 * license above.
 */
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

#define NUM_SECONDS   (1.)
#define SAMPLE_RATE   (44100)
#define FRAMES_PER_BUFFER  (256)
#define TS (44100)

#ifndef M_PI
#define M_PI  (3.14159265)
#endif

#define TABLE_SIZE   (200)
#define LENGTH (600)

struct syst {
	float a = 1.771508016511674e-06;
	float b = 1.760821058887331e-06;
	float c = -1.961859356989467;
	float d = 0.982022960467876;
} s;

struct timespec ts;

std::ofstream myfile;

namespace patch {
	// Patch to solve the non-existence of std::string::to_string()
    template <typename T> std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
} 

class Sine
{
public:
    Sine() : stream(0), left_phase(0), right_phase(0), k(0), j(0)
    {
        /* initialise sinusoidal wavetable */
        for( int i=0; i<TABLE_SIZE; i++ )
        {
            sine[i] = (float) sin( ((double)i/(double)TABLE_SIZE) * M_PI * 2. );
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
			
			float output = (s.a*ins[1] + s.b*ins[2])*SAMPLE_RATE - s.c*outs[1] - s.d*outs[2];

			outs[0] = output;
			func[i] = output;
			
			std::rotate(ins.rbegin(), ins.rbegin() + 1, ins.rend());
			std::rotate(outs.rbegin(), outs.rbegin() + 1, outs.rend());
			
			ins[0] = 0; // To make sure the initial dirac value disappears when 'ins' is rotated
			
			
        }

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
            printf("Output device name: '%s'\r", pInfo->name);
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
            paFramesPerBufferUnspecified,
            paClipOff,      /* we won't output out of range samples so don't bother clipping them */
            &Sine::paCallback,
            this            /* Using 'this' for userData so we can cast to Sine* in paCallback method */
            );

        if (err != paNoError)
        {
            /* Failed to open stream to device !!! */
            return false;
        }

        err = Pa_SetStreamFinishedCallback( stream, &Sine::paStreamFinished );

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

private:
    /* The instance callback, where we have access to every method/variable in object of class Sine */
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

        for( i=0; i<framesPerBuffer; i++ )
        {
			
			/*if (j < LENGTH) {
				myfile << patch::to_string(func[j]) << "\n";
			}*/
			
            *out++ = sine[left_phase];  /* left */
            *out++ = sine[right_phase];  /* right */
            //*out++ = func[j]; // The computed output needs to be sent through audio jack 
            left_phase += 2;
            if( left_phase >= TABLE_SIZE ) left_phase -= TABLE_SIZE;
            right_phase += 3; /* higher pitch so we can distinguish left and right. */
            if( right_phase >= TABLE_SIZE ) right_phase -= TABLE_SIZE;
            
            if (j < LENGTH-1) {
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
        /* Here we cast userData to Sine* type so we can call the instance method paCallbackMethod, we can do that since 
           we called Pa_OpenStream with 'this' for userData */
        return ((Sine*)userData)->paCallbackMethod(inputBuffer, outputBuffer,
            framesPerBuffer,
            timeInfo,
            statusFlags);
    }


    void paStreamFinishedMethod()
    {
        printf( "Stream Completed: %s\n", message );
    }

    /*
     * This routine is called by portaudio when playback is done.
     */
    static void paStreamFinished(void* userData)
    {
        return ((Sine*)userData)->paStreamFinishedMethod();
    }

    PaStream *stream;
    float sine[TABLE_SIZE];
    int left_phase;
    int right_phase;
    char message[20];
	std::vector<float> outs = std::vector<float>(10);
	std::vector<int> ins = std::vector<int>(10);
	float func[LENGTH];
	int k;
	int j;
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
int main(void);
int main(void)
{
	
	std::string s("ys.csv");
	myfile.open(s.c_str());
	
	ts.tv_sec = 0.;
	
    Sine sine;

    printf("PortAudio Test: output sine wave. SR = %d, BufSize = %d\n", SAMPLE_RATE, FRAMES_PER_BUFFER);
    
    ScopedPaHandler paInit;
    if( paInit.result() != paNoError ) goto error;

    if (sine.open(Pa_GetDefaultOutputDevice()))
    {
        if (sine.start())
        {
            printf("Play for %f seconds.\n", NUM_SECONDS );
            Pa_Sleep( NUM_SECONDS * 1000 );

            sine.stop();
        }

        sine.close();
    }

    printf("Test finished.\n");
    return paNoError;

error:
    fprintf( stderr, "An error occured while using the portaudio stream\n" );
    fprintf( stderr, "Error number: %d\n", paInit.result() );
    fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( paInit.result() ) );
    return 1;
}
